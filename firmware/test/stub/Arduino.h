// Host stand-in for the Arduino core: just what firmware/{chair,body} use,
// driven by the `sim` state from the tests (time, pins, ADC, USB serial).
#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <string>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define AR_VDD4 0
#define AR_INTERNAL_1_8 1
enum { D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, A0 = D0, LED_RED = 20, LED_GREEN, LED_BLUE };
// Body battery pins (variant.h numbers differ; these only must not collide)
#define VBAT_ENABLE 14
#define PIN_CHARGING_CURRENT 23
#define PIN_VBAT 32                            // analogRead only (not in sim::pin)

namespace sim {
extern uint32_t now;                           // millis()
extern int pin[32];
extern int adc;                                // every analogRead()
}

namespace sim {
inline int &vbat() { static int raw = 2997; return raw; }   // analogRead(PIN_VBAT); 2997 ~ 3.9 V
inline int rawForMv(int mv) { return (int)(mv / 1000.0 / 1.8 * 4096 * 510 / 1510 + 0.5); }
}

inline uint32_t millis() { return sim::now; }
inline uint32_t micros() { return sim::now * 1000u; }
inline void delayMicroseconds(unsigned) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int p, int v) { sim::pin[p] = v; }
inline int digitalRead(int p) { return sim::pin[p]; }
inline uint32_t &analogWriteCalls() { static uint32_t count = 0; return count; }
inline void analogWrite(int p, int v) { analogWriteCalls()++; sim::pin[p] = v; }
inline int analogRead(int p) { return p == PIN_VBAT ? sim::vbat() : sim::adc; }
inline void delay(uint32_t ms) { sim::now += ms; }
inline void analogReference(int) {}
inline void analogReadResolution(int) {}
inline void analogSampleTime(int) {}

// Adafruit_USBD_CDC as seen by the sketches + SerialOut: a `cap`-byte TX FIFO
// that only drains when the host reads. Writing more than fits is the case
// where the real write() would spin — counted in `blocked`.
struct FakeSerial {
  bool dtrOn = true;
  size_t cap = 256, used = 0;
  std::string out, in;                         // device -> host, host -> device
  int blocked = 0;
  void begin(unsigned long) {}
  explicit operator bool() { return dtrOn; }
  int dtr() { return dtrOn; }
  int availableForWrite() { return (int)(cap - used); }
  size_t write(const uint8_t *b, size_t n) {
    if (!dtrOn) return 0;
    if (used + n > cap) { blocked++; return 0; }
    used += n;
    out.append((const char *)b, n);
    return n;
  }
  int available() { return (int)in.size(); }
  int read() { if (in.empty()) return -1; int c = (unsigned char)in[0]; in.erase(0, 1); return c; }
  void hostReads() { used = 0; }
};
extern FakeSerial Serial;

// NRF_RNG (body's boot id): always ready, VALUE set by the test.
struct AlwaysReady {
  AlwaysReady &operator=(uint32_t) { return *this; }
  operator bool() const { return true; }
};
struct FakeRng { uint32_t TASKS_START, TASKS_STOP; AlwaysReady EVENTS_VALRDY; uint32_t VALUE; };
extern FakeRng *NRF_RNG;

// Core: RESETREAS saved (and cleared) at boot.
inline uint32_t &resetReasonSim() { static uint32_t r = 0; return r; }
inline uint32_t readResetReason() { return resetReasonSim(); }

// NRF_POWER: USB VBUS detect, and System OFF — which never returns on the
// board; here writing SYSTEMOFF throws SystemOff for the test to catch.
#define POWER_USBREGSTATUS_VBUSDETECT_Msk 1u
struct SystemOff {};
struct SystemOffReg { SystemOffReg &operator=(uint32_t v) { if (v) throw SystemOff(); return *this; } };
struct FakePower { uint32_t USBREGSTATUS; SystemOffReg SYSTEMOFF; };
inline FakePower &fakePower() { static FakePower p = {}; return p; }
#define NRF_POWER (&fakePower())

// NRF_WDT: configuration recorded; RR[0] == WDT_RR_RR_Reload after a feed.
#define WDT_CONFIG_HALT_Pos 3
#define WDT_CONFIG_HALT_Pause 0u
#define WDT_CONFIG_SLEEP_Pos 0
#define WDT_CONFIG_SLEEP_Run 1u
#define WDT_RREN_RR0_Msk 1u
#define WDT_RR_RR_Reload 0x6E524635u
#define WDT_RUNSTATUS_RUNSTATUS_Msk 1u
struct FakeWdt {
  uint32_t CONFIG, CRV, RREN, RUNSTATUS, RR[8];
  struct Start { FakeWdt *w; Start &operator=(uint32_t v) { if (v) w->RUNSTATUS = 1; return *this; } } TASKS_START{this};
};
inline FakeWdt &fakeWdt() { static FakeWdt w; return w; }
#define NRF_WDT (&fakeWdt())
