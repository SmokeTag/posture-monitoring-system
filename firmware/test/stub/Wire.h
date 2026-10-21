// Host stand-in for the core's TwoWire (Wire_nRF52.cpp) with an LSM6DS3TR-C
// on it at 0x6A: a register file + the faults the body must survive. Its
// contract is the real Wire's, not the Seeed driver's assumptions:
// requestFrom() returns the bytes actually delivered (as RXD.AMOUNT does),
// available()/read() give only those, endTransmission() returns 0 / 2 (address
// NACK) / 3 (data NACK) / 4 (other). What it can't show: a bus that hangs
// (the real Wire has no timeout and would never return).
//
// Accelerometer data: from sim::ax/ay/az (g, defined by each test), quantized
// as the ±16 g scale does (0.488 mg/LSB) — or raw[] verbatim with useRaw.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace sim { extern float ax, ay, az; }

struct FakeWire {
  // --- the sensor
  uint8_t regs[128];
  uint8_t ptr = 0;                             // register pointer (set by a write transaction)
  uint8_t addr = 0x6A;
  void (*onTransaction)() = nullptr;          // optional assertion of actuator safety
  bool begun = false;                          // begin() called (the real TWIM is off before)

  // --- fault injection (persist until cleared)
  bool nackAll = false;                        // nobody answers: endTransmission 2, requestFrom 0
  bool nackData = false;                       // writes: data byte NACKed (endTransmission 3)
  bool writeFails = false;                     // write() refuses bytes (tx buffer full)
  int shortReg = -1;                           // reads starting at this register...
  uint8_t shortGive = 0;                       // ...deliver only this many bytes (0 = none)
  bool lieCount = false;                       // ...and requestFrom() still claims the full count
  int stuckReg = -1;                           // writes to this register are ignored (readback differs)
  bool forceStale = false;                     // STATUS_REG.XLDA stays 0
  int statusOverride = -1;                     // STATUS_REG reads this value instead (0..255)
  bool useRaw = false;                         // OUTX..OUTZ come from raw[] instead of sim::a*
  int16_t raw[3] = {0, 0, 0};

  // --- what happened
  int tx = 0, rx = 0, regWrites = 0, unbegun = 0;

  FakeWire() { reset(); }

  // Sensor power-on / brownout reset: register defaults (accelerometer in
  // power-down, IF_INC set), WHO_AM_I 0x6A.
  void reset() {
    memset(regs, 0, sizeof regs);
    regs[0x0F] = 0x6A;                         // WHO_AM_I
    regs[0x12] = 0x04;                         // CTRL3_C: IF_INC
    ptr = 0;
    rxBuf_.n = rxBuf_.pos = 0;
  }

  // --- TwoWire API used by Lsm6Accel
  void begin() { begun = true; }
  void end() { begun = false; }
  void beginTransmission(uint8_t a) { if (onTransaction) onTransaction(); txAddr_ = a; txLen_ = 0; txOn_ = true; }
  size_t write(uint8_t b) {
    if (!txOn_ || writeFails || txLen_ >= sizeof txBuf_) return 0;
    txBuf_[txLen_++] = b;
    return 1;
  }
  uint8_t endTransmission(bool stop = true) {
    (void)stop;
    tx++;
    txOn_ = false;
    if (!begun) { unbegun++; return 4; }
    if (nackAll || txAddr_ != addr) return 2;
    if (txLen_ == 0) return 0;
    ptr = txBuf_[0];
    if (txLen_ > 1 && nackData) return 3;
    for (uint8_t i = 1; i < txLen_; i++) {
      uint8_t r = (uint8_t)(ptr + i - 1) & 0x7F;
      if ((int)r != stuckReg) regs[r] = txBuf_[i];
      regWrites++;
    }
    return 0;
  }
  uint8_t requestFrom(uint8_t a, size_t n, bool stop = true) {
    (void)stop;
    rx++;
    rxBuf_.n = rxBuf_.pos = 0;
    if (!begun) { unbegun++; return 0; }
    if (n == 0 || nackAll || a != addr) return 0;
    if (n > sizeof rxBuf_.b) n = sizeof rxBuf_.b;   // the real rx buffer: 64 bytes
    size_t give = (int)ptr == shortReg && shortGive < n ? shortGive : n;
    bool inc = regs[0x12] & 0x04;              // CTRL3_C.IF_INC
    for (size_t i = 0; i < give; i++) rxBuf_.b[i] = regAt((uint8_t)(inc ? ptr + i : ptr));
    rxBuf_.n = give;
    return (uint8_t)(lieCount ? n : give);
  }
  int available() { return (int)(rxBuf_.n - rxBuf_.pos); }
  int read() { return rxBuf_.pos < rxBuf_.n ? rxBuf_.b[rxBuf_.pos++] : -1; }

 private:
  static int16_t toRaw(float g) {
    float v = g / 0.000488f;
    v = v < 0 ? v - 0.5f : v + 0.5f;
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
  }
  uint8_t regAt(uint8_t r) {
    r &= 0x7F;
    if (r == 0x1E) {                           // STATUS_REG
      if (statusOverride >= 0) return (uint8_t)statusOverride;
      return (!forceStale && (regs[0x10] & 0xF0)) ? 0x01 : 0x00;   // XLDA while ODR != off
    }
    if (r >= 0x28 && r <= 0x2D) {              // OUTX_L_XL .. OUTZ_H_XL
      int i = (r - 0x28) / 2;
      int16_t v = useRaw ? raw[i] : toRaw(i == 0 ? sim::ax : i == 1 ? sim::ay : sim::az);
      return (r - 0x28) % 2 ? (uint8_t)((uint16_t)v >> 8) : (uint8_t)(v & 0xFF);
    }
    return regs[r];
  }

  uint8_t txAddr_ = 0, txLen_ = 0;
  bool txOn_ = false;
  uint8_t txBuf_[32];
  struct { uint8_t b[64]; size_t n, pos; } rxBuf_ = {};
};

typedef FakeWire TwoWire;
extern FakeWire Wire1;                         // the IMU bus on the XIAO Sense (defined by each test)
