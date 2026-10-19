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
enum { D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, A0 = D0, LED_RED = 20, LED_GREEN, LED_BLUE };

namespace sim {
extern uint32_t now;                           // millis()
extern int pin[32];
extern int adc;                                // every analogRead()
}

inline uint32_t millis() { return sim::now; }
inline uint32_t micros() { return sim::now * 1000u; }
inline void delayMicroseconds(unsigned) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int p, int v) { sim::pin[p] = v; }
inline int digitalRead(int p) { return sim::pin[p]; }
inline void analogWrite(int p, int v) { sim::pin[p] = v; }
inline int analogRead(int) { return sim::adc; }
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
