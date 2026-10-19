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
 * Link state: the body counts the link as up only while VALID ChairAcks keep
 * arriving (BodyLink); a bare/empty/odd ACK proves the radio got through, not
 * that the chair is in control. The chair tracks the body's seq per boot
 * session (SeqTracker), so a body reboot is not counted as lost packets.
 *
 * Build: arduino-cli ... --libraries firmware/libraries (see firmware/README.md).
 */
#pragma once
#if defined(ARDUINO)
#include <Arduino.h>
#else                                          // host unit tests (firmware/test)
#include <stdint.h>
#include <stddef.h>
#endif
#include <math.h>
#include <string.h>

namespace posture {

// --- ESB link ---
const uint8_t RF_ADDR[6] = "PCHR1";           // 5-byte pipe address
const uint8_t RF_CHANNEL = 76;                // 2476 MHz
const uint8_t RF_PIPE    = 1;                 // chair's reading pipe (and ACK-payload pipe)
const unsigned long LINK_TIMEOUT_MS = 1000;   // no packet/ACK this long = link down

enum : uint8_t { BF_MOTOR = 1 << 0 };          // BodyPacket.flags: motor/LED on right now
struct __attribute__((packed)) BodyPacket {
  uint16_t seq;                                // +1 per sample (wraps 65535 -> 0)
  uint8_t  flags;                              // BF_*
  uint8_t  boot;                               // random 1..255 per body boot; 0 = unknown
                                               // (older body firmware: was `reserved`, always 0)
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

// --- Body side: freshness of the chair's control state ---

inline bool finiteF(float v) { return v == v && v - v == 0.0f; }   // not NaN, not +-inf

// A ChairAck is only trusted if it has the exact size and sane contents.
inline bool parseChairAck(const void *buf, uint8_t len, ChairAck &out) {
  if (len != sizeof(ChairAck)) return false;
  ChairAck a;
  memcpy(&a, buf, sizeof(a));
  if (a.flags & ~(CF_CAL | CF_ALERT)) return false;
  if ((a.flags & CF_CAL) && !(finiteF(a.pitchRef) && finiteF(a.rollRef))) return false;
  out = a;
  return true;
}

// Up while fresh control data keeps arriving; down LINK_TIMEOUT_MS after the
// last one. Call update() on every sample: going down by state (not by
// comparing timestamps later) keeps it right across the millis() rollover.
struct LinkWatch {
  bool up = false;
  uint32_t lastAt = 0;

  // Returns true when `up` changed.
  bool update(uint32_t t, bool fresh) {
    bool was = up;
    if (fresh) { lastAt = t; up = true; }
    else if (up && t - lastAt >= LINK_TIMEOUT_MS) up = false;
    return up != was;
  }
};

// What one radio.write() and its ACK payload mean to the body. Transport
// (ACK received) and control (a valid ChairAck in it) are counted apart; only
// control renews the link. `ack` keeps the last valid state across link loss,
// so the fallback still has the chair's reference.
struct BodyLink {
  ChairAck ack = {};
  bool haveAck = false;                        // ever got a valid ChairAck
  uint8_t lastBuzz = 0;
  LinkWatch watch;
  // since clearCounts(); noack = sent - acked; acked = ctl + bad
  uint32_t sent = 0, acked = 0, retx = 0, ctl = 0, bad = 0;

  struct Result { bool ctl, buzz, linkChanged; };

  // ok/arc: radio.write() result and getARC() (retransmissions of this write,
  // also when it ran out of retries). payload/len: the ACK payload (len 0 if none).
  Result onTx(uint32_t t, bool ok, uint8_t arc, const void *payload, uint8_t len) {
    Result r = {false, false, false};
    sent++;
    retx += arc;
    ChairAck a;
    if (ok) {
      acked++;
      if (parseChairAck(payload, len, a)) {
        ctl++;
        r.ctl = true;
        r.buzz = watch.up && a.buzz != lastBuzz;   // a fresh link only syncs the counter
        lastBuzz = a.buzz;
        ack = a;
        haveAck = true;
      } else {
        bad++;
      }
    }
    r.linkChanged = watch.update(t, r.ctl);
    return r;
  }

  void clearCounts() { sent = acked = retx = ctl = bad = 0; }
};

// --- Chair side: the body's packet sequence ---

enum SeqKind : uint8_t {
  SEQ_FIRST,                                   // first packet since chair boot
  SEQ_NEXT,                                    // in order (gap, if any, counted in `lost`)
  SEQ_RESUME,                                  // same session after a link outage: gap not counted
  SEQ_RESTART,                                 // new body session (boot id changed, or seq
                                               // went back / jumped with no other explanation)
  SEQ_DUP,                                     // same seq again: ignore
  SEQ_OLD,                                     // slightly behind, same session, link up: ignore
                                               // (DUP/OLD are no new sample: they must not
                                               // keep the chair's link up)
};
const uint16_t SEQ_MAX_GAP = 64;               // ~3 s at 20 Hz; while linked, gaps are < ~20

// lost = seq numbers skipped between two packets of the same session received
// less than LINK_TIMEOUT_MS apart. Packets missed during an outage are not
// counted (the outage itself is the "# body link lost/up" events), nor across a
// restart. Wrap 65535 -> 0 is an ordinary step.
// Reboots, best effort: the boot id is 1 random byte, so a reboot keeps the
// same id with probability 1/255 (and boot = 0, old body firmware, never
// tells). Then it is inferred from seq: back after an outage -> RESTART (the
// usual case: the body's boot takes ~1 s+); but if it lands within the same
// link window it can pass as OLD/DUP (samples ignored until the link times
// out, then RESTART) or as NEXT/RESUME (a few bogus `lost`). No exactness is
// claimed; a wider id would need a protocol change.
struct SeqTracker {
  bool have = false;
  uint8_t boot = 0;
  uint16_t seq = 0;
  uint32_t at = 0;                             // millis() of the last accepted packet
  uint32_t pkts = 0, lost = 0, dup = 0, old = 0, rst = 0;   // since clearCounts()

  SeqKind onPacket(uint8_t b, uint16_t s, uint32_t t) {
    uint16_t fwd = s - seq, back = seq - s;
    bool linked = t - at < LINK_TIMEOUT_MS;
    SeqKind k;
    if (!have)                       k = SEQ_FIRST;
    else if (b != boot)              k = SEQ_RESTART;
    else if (fwd == 0)               k = SEQ_DUP;
    else if (fwd >= 0x8000)          k = (b && linked && back <= SEQ_MAX_GAP) ? SEQ_OLD : SEQ_RESTART;
    else if (!linked)                k = SEQ_RESUME;
    else if (fwd > SEQ_MAX_GAP)      k = SEQ_RESTART;
    else                             k = SEQ_NEXT;

    if (k == SEQ_DUP) { dup++; return k; }
    if (k == SEQ_OLD) { old++; return k; }
    if (k == SEQ_NEXT) lost += fwd - 1;
    if (k == SEQ_RESTART) rst++;
    have = true; boot = b; seq = s; at = t;
    pkts++;
    return k;
  }

  void clearCounts() { pkts = lost = dup = old = rst = 0; }
};

}  // namespace posture
