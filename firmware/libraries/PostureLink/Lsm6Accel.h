/*
 * Lsm6Accel — checked accelerometer reads from the body's LSM6DS3TR-C
 * -------------------------------------------------------------------
 * Why not the Seeed Arduino LSM6DS3 driver (2.0.5): readRegisterRegion()
 * returns IMU_SUCCESS even when Wire.requestFrom() delivered fewer bytes than
 * asked (it never checks the count); readRawAccel*() then return an
 * uninitialized value on failure, and readFloatAccel*() hide the status
 * altogether. beginCore() checks WHO_AM_I into an uninitialized byte without
 * the read's status, and begin() ignores every config write. So a sample from
 * it can be invented or old with nothing saying so.
 *
 * What this does instead (registers: ST AN5130 / DS LSM6DS3TR-C):
 *   begin(): WHO_AM_I == 0x6A, then CTRL3_C = 0x44 (BDU: output registers
 *     not updated between reading the low and the high byte; IF_INC: burst
 *     reads auto-increment), CTRL2_G = 0x00 (gyro power-down: not used; also
 *     clears whatever a previous firmware left there — the sensor may stay
 *     powered across an MCU reset), CTRL1_XL = 0x66 (the byte the Seeed
 *     begin() wrote: ODR 416 Hz, ±16 g, LPF1_BW_SEL — angles stay comparable
 *     to earlier captures). Then reads the three back and compares.
 *   read(): STATUS_REG.XLDA must say a new sample is there (else STALE), then
 *     one 6-byte burst from OUTX_L_XL. Every transaction is checked:
 *     endTransmission() == 0, requestFrom() == n AND available() == n, every
 *     read() >= 0; leftover bytes are drained.
 *   Plausibility (RANGE): an axis at full scale (-32768 / 32767), or |a| <
 *     0.25 g. That catches all-zero / all-ones words, but also real motion or
 *     free fall: it means "no usable gravity direction in this sample", not
 *     "the chip is broken". Identical consecutive values are NOT treated as
 *     stale: a body sitting still reads the same counts.
 *
 * Limits (stated, not solved here):
 *   - Wire_nRF52.cpp (core 1.1.13) has no timeout: endTransmission() and
 *     requestFrom() spin on TWIM events. A NACK returns (handled: BUS), but a
 *     bus held low with no NACK/ERROR event blocks inside Wire, and this class
 *     can't bound that.
 *   - XLDA proves the sensor produced a new sample since the last read, not
 *     that its contents are right: a sensor that keeps setting XLDA over
 *     frozen or wrong data passes, as does a plausible-looking corrupt word.
 *   - begin() == IMU_OK means "this chip answered as an LSM6DS3TR-C and holds
 *     our configuration now". A later reset of the sensor alone (brownout)
 *     returns it to power-down: XLDA stops, read() says STALE; re-begin() to
 *     recover (the caller decides when).
 * No GPIO, delay, retry or state here: power-up, retry policy and the motor
 * are the caller's (firmware/body). Bus: anything shaped like TwoWire (Wire1
 * on the XIAO Sense; FakeWire in firmware/test/stub/Wire.h).
 */
#pragma once
#include <PostureLink.h>                       // IMU_OK, IMU_INIT, IMU_BUS, IMU_STALE, IMU_RANGE
#include <stdint.h>
#include <stddef.h>

namespace posture {

namespace lsm6 {
const uint8_t ADDR         = 0x6A;             // SA0 high on the XIAO Sense
const uint8_t WHO_AM_I     = 0x0F;
const uint8_t WHO_AM_I_VAL = 0x6A;             // LSM6DS3TR-C (plain LSM6DS3: 0x69)
const uint8_t CTRL1_XL     = 0x10;
const uint8_t CTRL2_G      = 0x11;
const uint8_t CTRL3_C      = 0x12;
const uint8_t STATUS_REG   = 0x1E;
const uint8_t OUTX_L_XL    = 0x28;             // X L/H, Y L/H, Z L/H

const uint8_t CTRL1_XL_VAL = 0x66;             // ODR 416 Hz | FS ±16 g | LPF1_BW_SEL
const uint8_t CTRL2_G_VAL  = 0x00;             // gyro power-down
const uint8_t CTRL3_C_VAL  = 0x44;             // BDU | IF_INC
const uint8_t XLDA         = 0x01;             // STATUS_REG: new accelerometer sample

const float G_PER_LSB = 0.000488f;             // ±16 g: 0.488 mg/LSB
const int32_t MIN_NORM_LSB = 512;              // 0.25 g
}  // namespace lsm6

template <class Bus>
class Lsm6Accel {
 public:
  explicit Lsm6Accel(Bus &bus, uint8_t addr = lsm6::ADDR) : bus_(bus), addr_(addr) {}

  // IMU_OK: identified and configured (read back). IMU_INIT: it answered, but
  // not as an LSM6DS3TR-C, or it doesn't hold the configuration. IMU_BUS: an
  // I2C transaction failed. Up to 5 transactions.
  uint8_t begin() {
    uint8_t id;
    if (!readRegs(lsm6::WHO_AM_I, &id, 1)) return IMU_BUS;
    whoAmI = id;
    if (id != lsm6::WHO_AM_I_VAL) return IMU_INIT;
    if (!writeReg(lsm6::CTRL3_C, lsm6::CTRL3_C_VAL) || !writeReg(lsm6::CTRL2_G, lsm6::CTRL2_G_VAL) ||
        !writeReg(lsm6::CTRL1_XL, lsm6::CTRL1_XL_VAL))
      return IMU_BUS;
    uint8_t c[3];                              // CTRL1_XL, CTRL2_G, CTRL3_C (IF_INC now set)
    if (!readRegs(lsm6::CTRL1_XL, c, 3)) return IMU_BUS;
    if (c[0] != lsm6::CTRL1_XL_VAL || c[1] != lsm6::CTRL2_G_VAL || c[2] != lsm6::CTRL3_C_VAL)
      return IMU_INIT;
    return IMU_OK;
  }

  // One new sample, in g. IMU_OK only if every byte arrived, it is new
  // (XLDA) and plausible; otherwise IMU_BUS / IMU_STALE / IMU_RANGE and
  // ax/ay/az are left untouched. Up to 2 transactions (status, data).
  uint8_t read(float &ax, float &ay, float &az) {
    uint8_t st;
    if (!readRegs(lsm6::STATUS_REG, &st, 1)) return IMU_BUS;
    if (!(st & lsm6::XLDA)) return IMU_STALE;
    uint8_t b[6];
    if (!readRegs(lsm6::OUTX_L_XL, b, 6)) return IMU_BUS;
    int16_t x = (int16_t)(b[0] | (b[1] << 8));
    int16_t y = (int16_t)(b[2] | (b[3] << 8));
    int16_t z = (int16_t)(b[4] | (b[5] << 8));
    if (fullScale(x) || fullScale(y) || fullScale(z)) return IMU_RANGE;
    uint32_t n2 = (uint32_t)((int32_t)x * x) + (uint32_t)((int32_t)y * y) + (uint32_t)((int32_t)z * z);
    if (n2 < (uint32_t)(lsm6::MIN_NORM_LSB * lsm6::MIN_NORM_LSB)) return IMU_RANGE;
    ax = x * lsm6::G_PER_LSB;
    ay = y * lsm6::G_PER_LSB;
    az = z * lsm6::G_PER_LSB;
    return IMU_OK;
  }

  uint8_t whoAmI = 0;                          // last WHO_AM_I read by begin() (for a log line)

 private:
  static bool fullScale(int16_t v) { return v == INT16_MIN || v == INT16_MAX; }

  // n bytes from register `reg` (n > 1 relies on IF_INC). False on any error
  // or short count; never leaves bytes behind for the next transaction.
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
    bus_.beginTransmission(addr_);
    bool ok = bus_.write(reg) == 1;
    ok = bus_.endTransmission() == 0 && ok;
    if (ok) {
      uint8_t got = bus_.requestFrom(addr_, (size_t)n);
      ok = got == n && bus_.available() == (int)n;
      for (uint8_t i = 0; ok && i < n; i++) {
        int c = bus_.read();
        if (c < 0) ok = false;
        else buf[i] = (uint8_t)c;
      }
    }
    for (int i = 0; i < 64 && bus_.available() > 0; i++) bus_.read();   // Wire's rx buffer: 64
    return ok;
  }

  bool writeReg(uint8_t reg, uint8_t v) {
    bus_.beginTransmission(addr_);
    bool ok = bus_.write(reg) == 1 && bus_.write(v) == 1;
    return bus_.endTransmission() == 0 && ok;
  }

  Bus &bus_;
  uint8_t addr_;
};

}  // namespace posture
