/*
 * PostureLink — shared by firmware/chair (the brain) and firmware/body
 * --------------------------------------------------------------------
 * One source of truth for the ESB link and the posture alert rule, so the
 * chair's decision and the body's link-loss fallback can't drift apart.
 *
 * Link (Nordic ESB, nrf_to_nrf): body = transmitter, chair = receiver.
 *   body  -> chair : BodyPacket, one per IMU sample (20 Hz)
 *   chair -> body  : ChairAck, as the ESB ACK payload of that packet. The chair
 *                    preloads it; nrf_to_nrf resends the same buffer on every
 *                    ACK until it is rewritten, so it always carries the latest
 *                    state (one packet behind, i.e. <= 50 ms).
 *
 * Build: arduino-cli ... --libraries firmware/libraries (see firmware/README.md).
 */
#pragma once
#include <Arduino.h>
#include <math.h>

namespace posture {

// --- ESB link ---
const uint8_t RF_ADDR[6] = "PCHR1";           // 5-byte pipe address
const uint8_t RF_CHANNEL = 76;                // 2476 MHz
const uint8_t RF_PIPE    = 1;                 // chair's reading pipe (and ACK-payload pipe)
const unsigned long LINK_TIMEOUT_MS = 1000;   // no packet/ACK this long = link down

enum : uint8_t { BF_MOTOR = 1 << 0 };          // BodyPacket.flags: motor/LED on right now
struct __attribute__((packed)) BodyPacket {
  uint16_t seq;                                // +1 per sample; gaps = lost packets
  uint8_t  flags;                              // BF_*
  uint8_t  reserved;
  float    pitch, roll;                        // degrees, from the accelerometer
};

enum : uint8_t {                               // ChairAck.flags
  CF_CAL   = 1 << 0,                           // reference valid (pitchRef/rollRef)
  CF_ALERT = 1 << 1,                           // chair says: alert now
};
struct __attribute__((packed)) ChairAck {
  uint8_t  flags;                              // CF_*
  uint8_t  buzz;                               // +1 per motor-test request (body buzzes on change)
  uint16_t reserved;
  float    pitchRef, rollRef;                  // calibrated upright, for the body's fallback
};

// --- Posture alert ---
const float ALERT_DEG = 10.0f;                 // analysis/archive/VALIDATION.md: upright p95 6.4°, bad p05 18.6°
const float CLEAR_DEG = 7.0f;                  // hysteresis: alert ends below this
const unsigned long ALERT_HOLD_MS = 3000;      // dev must stay > ALERT_DEG this long

// Degrees away from the calibrated upright reference.
inline float deviation(float pitch, float roll, float pitchRef, float rollRef) {
  float dp = pitch - pitchRef, dr = roll - rollRef;
  return sqrtf(dp * dp + dr * dr);
}

// Over ALERT_DEG for ALERT_HOLD_MS -> alert; below CLEAR_DEG -> clear.
struct AlertGate {
  bool alert = false;
  unsigned long overSince = 0;                 // 0: not over threshold

  void reset() { alert = false; overSince = 0; }

  // Returns true when `alert` changed.
  bool update(float dev, unsigned long t) {
    if (dev > ALERT_DEG) {
      if (!overSince) overSince = t ? t : 1;
      if (!alert && t - overSince >= ALERT_HOLD_MS) { alert = true; return true; }
    } else {
      overSince = 0;
      if (alert && dev < CLEAR_DEG) { alert = false; return true; }
    }
    return false;
  }
};

}  // namespace posture
