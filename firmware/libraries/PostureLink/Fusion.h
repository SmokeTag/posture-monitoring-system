/*
 * Fusion — the chair's posture alert: body IMU + the chair's FSRs
 * ---------------------------------------------------------------
 * Pure logic (no Arduino calls), 1:1 with decide() + Gate in
 * analysis/fusion.py (same thresholds, order and reason codes; derivation
 * and evaluation: docs/decisao-fusao-imu-fsr.md). Only the chair runs it; the
 * body's link-loss fallback stays IMU-only (AlertGate in PostureLink.h).
 *
 * Inputs per body sample: dpitch/droll (deg from the calibrated reference;
 * with the collar mount droll is sagittal, + = backwards, and dpitch lateral,
 * + = right), the FSRs (raw ADC counts) by spot, and the FSR baseline: the
 * mean of each FSR over the same CAL_SAMPLES as the IMU reference.
 *
 *   seat < EMPTY_RAW                          -> EMPTY (no-one seated: reset, no alert)
 *   rear = seat rear / its baseline (1 if that baseline < BASE_MIN)
 *   rear < SLUMP_FRAC                         -> SLUMP (hips slid forward)
 *   droll > 0 and rear >= RECLINE_FRAC        -> backward tilt ignored (reclined)
 *   dev = |(dpitch, droll or 0)| > ALERT_DEG  -> IMU_DEV
 *   |dpitch| >= LEAN_DEG and the seat's L/R balance moved >= LEAN_LR that way -> LEAN
 *   ok (may clear) = none of those and dev < CLEAR_DEG
 * FusionGate: bad for ALERT_HOLD_MS -> alert, an ok sample clears it (as AlertGate).
 */
#pragma once
#include <stdint.h>
#include <math.h>
#include "FloatBits.h"
#include "PostureLink.h"                       // ALERT_DEG, CLEAR_DEG, ALERT_HOLD_MS

namespace posture {

// Chair spots (analysis/chair_map.py SLOTS; sitter's left/right).
enum Spot : uint8_t {
  SEAT_FRONT_LEFT, SEAT_FRONT_RIGHT, SEAT_REAR_LEFT, SEAT_REAR_RIGHT,
  BACK_LOWER_LEFT, BACK_LOWER_RIGHT, BACK_UPPER_LEFT, BACK_UPPER_RIGHT, N_SPOTS
};

// Reason codes (fusion.py CODES); the chair CSV `reason` column, -1 = no decision.
enum Reason : uint8_t { R_OK, R_IMU_DEV, R_SLUMP, R_LEAN, R_RECLINE_OK, R_EMPTY, R_INVALID };

const float SLUMP_FRAC   = 0.3f;               // seat rear / baseline below: slid forward
const float RECLINE_FRAC = 0.7f;               // at least: hips back, droll > 0 ignored
const float LEAN_DEG     = 5.0f;               // |dpitch| for an FSR-confirmed lean
const float LEAN_LR      = 0.08f;              // seat (L-R)/(L+R) shift vs baseline, towards it
const float EMPTY_RAW    = 2000.0f;            // seat sum (4 FSRs, raw) below: no-one seated
const float BASE_MIN     = 1000.0f;            // a 2-sensor baseline below this is unusable

// Captured at calibration from the per-spot FSR means.
struct FsrBaseline {
  float rear = 0, lr = 0;                      // seat rear sum (raw); seat (L-R)/(L+R)
  bool rearOk = false, lrOk = false;           // false: those terms are off (IMU only)

  static FsrBaseline of(const float m[N_SPOTS]) {
    FsrBaseline b;
    b.rear = m[SEAT_REAR_LEFT] + m[SEAT_REAR_RIGHT];
    float l = m[SEAT_FRONT_LEFT] + m[SEAT_REAR_LEFT], r = m[SEAT_FRONT_RIGHT] + m[SEAT_REAR_RIGHT];
    b.lrOk = l + r >= 2 * BASE_MIN;
    b.lr = b.lrOk ? (l - r) / (l + r) : 0.0f;
    b.rearOk = b.rear >= BASE_MIN;
    return b;
  }
};

struct Decision {
  uint8_t reason;                              // Reason
  bool bad;                                    // counts towards the alert hold
  bool ok;                                     // an active alert may clear
};

// One sample, no state. f = FSRs by spot (raw counts).
inline Decision decide(float dp, float dr, const float f[N_SPOTS], const FsrBaseline &b) {
  if (!floatIsFinite(dp) || !floatIsFinite(dr)) return {R_INVALID, false, false};
  float seat = f[SEAT_FRONT_LEFT] + f[SEAT_FRONT_RIGHT] + f[SEAT_REAR_LEFT] + f[SEAT_REAR_RIGHT];
  if (seat < EMPTY_RAW) return {R_EMPTY, false, false};
  float rear = b.rearOk ? (f[SEAT_REAR_LEFT] + f[SEAT_REAR_RIGHT]) / b.rear : 1.0f;
  bool slump = b.rearOk && rear < SLUMP_FRAC;
  bool recline = b.rearOk && dr > 0 && rear >= RECLINE_FRAC;
  float drEff = recline ? 0.0f : dr;
  float dev = sqrtf(dp * dp + drEff * drEff);
  bool lean = false;
  if (b.lrOk && fabsf(dp) >= LEAN_DEG) {
    float lr = (f[SEAT_FRONT_LEFT] + f[SEAT_REAR_LEFT] - f[SEAT_FRONT_RIGHT] - f[SEAT_REAR_RIGHT])
               / seat - b.lr;                  // + = load moved left
    lean = (dp < 0 ? lr : -lr) >= LEAN_LR;     // dpitch < 0 = leaning left
  }
  if (slump) return {R_SLUMP, true, false};
  if (dev > ALERT_DEG) return {R_IMU_DEV, true, false};
  if (lean) return {R_LEAN, true, false};
  uint8_t r = recline && sqrtf(dp * dp + dr * dr) > ALERT_DEG ? R_RECLINE_OK : R_OK;
  return {r, false, dev < CLEAR_DEG};
}

// bad for ALERT_HOLD_MS -> alert; while alerted an ok sample clears it; a
// sample neither bad nor ok holds the state and restarts the hold.
struct FusionGate {
  bool alert = false;
  unsigned long overSince = 0;                 // 0: not bad

  void reset() { alert = false; overSince = 0; }

  // Returns true when `alert` changed.
  bool update(bool bad, bool ok, unsigned long t) {
    if (bad) {
      if (!overSince) overSince = t ? t : 1;
      if (!alert && t - overSince >= ALERT_HOLD_MS) { alert = true; return true; }
    } else {
      overSince = 0;
      if (alert && ok) { alert = false; return true; }
    }
    return false;
  }
};

}  // namespace posture
