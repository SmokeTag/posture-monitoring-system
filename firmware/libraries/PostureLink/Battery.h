/*
 * Battery — body unit's 1S LiPo: voltage reading + low-voltage cutoff rule
 * ------------------------------------------------------------------------
 * Pure logic (no Arduino calls) so the host tests drive it; body.ino does the
 * ADC reads, the USB check and the System OFF. Cell: JYH LIP103450-1800
 * (1800 mAh, discharge cutoff 2.75 V); research in docs/pesquisa-bateria-lipo.md.
 *
 * Reading: P0.31 (AIN7) sits on a 1 MΩ / 510 kΩ divider, enabled by P0.14
 * LOW (never HIGH). ADC: internal 0.6 V reference with gain 1/3
 * (AR_INTERNAL_1_8: 0..1.8 V, independent of the 3V3 rail), 12 bit, 40 µs
 * acquisition (the divider's ~340 kΩ source impedance needs it). 4.2 V gives
 * ~1.42 V at the pin, inside the range with margin.
 *   Vbat = raw * 1.8 V / 4096 * VBAT_DIVIDER
 * VBAT_DIVIDER is the nominal 1510/510; resistor and reference tolerances add
 * a few %. Calibrate on the bench: multimeter on BAT+/BAT- vs the reported
 * vbat, new VBAT_DIVIDER = 2.961 * Vmeter / Vreported.
 *
 * Cutoff: 3.30 V leaves margin above the cell's 2.75 V and the XIAO's 3V3
 * regulator dropout; the cell is nearly empty there (the curve falls fast
 * below ~3.4 V). A reading is the mean of VBAT_SAMPLES conversions, taken
 * only after the motor has been off VBAT_MOTOR_REST_MS (its ~90 mA sags the
 * cell); VBAT_LOW_READS consecutive readings below the cutoff trip it (one
 * noisy reading can't), a reading above VBAT_CLEAR_MV restarts the count,
 * and in between it holds. Never on USB (the board runs from VBUS then and
 * the charger sets VBAT). Readings below VBAT_VALID_MV can't come from a
 * cell that powers the board: a measurement fault, ignored (not a cutoff).
 */
#pragma once
#include <stdint.h>

namespace posture {

const float    VBAT_ADC_FS_V  = 1.8f;          // AR_INTERNAL_1_8
const uint32_t VBAT_ADC_COUNTS = 4096;         // 12 bit
const float    VBAT_DIVIDER   = 1510.0f / 510.0f;   // ~2.961, nominal; calibrate (above)
const uint8_t  VBAT_SAMPLES   = 16;            // conversions per reading (~1 ms)

const uint16_t VBAT_CUTOFF_MV = 3300;          // below: counts towards the cutoff
const uint16_t VBAT_CLEAR_MV  = 3350;          // above: the count restarts
const uint16_t VBAT_BOOT_MV   = 3400;          // a boot on battery needs at least this
const uint16_t VBAT_VALID_MV  = 2000;          // below: measurement fault, ignored
const uint8_t  VBAT_LOW_READS = 3;             // consecutive low readings that trip it
const uint32_t VBAT_PERIOD_MS = 2000;          // one reading every 2 s...
const uint32_t VBAT_MOTOR_REST_MS = 200;       // ...once the motor has been off this long

// Mean of `count` raw 12-bit conversions -> battery millivolts.
inline uint16_t vbatMv(uint32_t rawSum, uint8_t count) {
  if (!count) return 0;
  float v = (float)rawSum / count * VBAT_ADC_FS_V / VBAT_ADC_COUNTS * VBAT_DIVIDER;
  return v >= 65.535f ? 65535 : (uint16_t)(v * 1000.0f + 0.5f);
}

// Boot check: on battery (no USB) and already below VBAT_BOOT_MV -> straight
// back to System OFF, so a reset or a USB-unplug wake can't run the cell flat.
// The margin over the cutoff keeps a cell that recovered a little at rest
// from booting just to trip again.
inline bool bootLow(uint16_t mv, bool usb) {
  return !usb && mv >= VBAT_VALID_MV && mv < VBAT_BOOT_MV;
}

struct BatteryMonitor {
  uint16_t mv = 0;                             // last reading; 0 = none yet
  uint8_t lows = 0;                            // consecutive readings below the cutoff
  bool have = false;
  uint32_t lastAt = 0;                         // millis() of the last reading

  // A reading is due every VBAT_PERIOD_MS, once the motor has rested.
  bool due(uint32_t t, bool motorOn, uint32_t motorOffAt) const {
    if (motorOn || t - motorOffAt < VBAT_MOTOR_REST_MS) return false;
    return !have || t - lastAt >= VBAT_PERIOD_MS;
  }

  // One reading; true = cut off now.
  bool update(uint32_t t, uint16_t reading, bool usb) {
    have = true;
    lastAt = t;
    mv = reading;
    if (usb || reading > VBAT_CLEAR_MV) lows = 0;
    else if (reading >= VBAT_VALID_MV && reading < VBAT_CUTOFF_MV && lows < VBAT_LOW_READS) lows++;
    return lows >= VBAT_LOW_READS;
  }
};

}  // namespace posture
