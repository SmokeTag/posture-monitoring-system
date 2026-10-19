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
 *                    state (one packet behind).
 *
 * Link state: the body counts the link as up only while VALID ChairAcks keep
 * arriving (BodyLink); a bare/empty/odd ACK proves the radio got through, not
 * that the chair is in control. The chair tracks the body's seq per boot
 * session (SeqTracker), so a body reboot is not counted as lost packets.
 *
 * Telemetry: the two boards' millis() are never compared; a decision is tied
 * to the body sample it was made on (ChairAck.basis*), and its age is counted
 * in body samples. With no loss, for the decision made on body sample N:
 *   chair, row seq=N    : decides (cal/calid/alert), preloads ChairAck basis N
 *   body,  packet N+1   : sent BEFORE its ACK arrives -> still reports the state
 *                         it applied at N (basis N-1, age 2)
 *   body,  row seq=N+1  : AFTER that ACK -> applies basis N (age 1)
 *   chair, row seq=N+2  : packet N+2 reports what the body applied at N+1:
 *                         bseq = N, bage = 2
 * Ids are best effort: the body boot id and the chair's calId are 1 byte
 * each, so a reboot can reuse one (1/255); the checks in BodyLink keep a reused
 * boot id from passing as a fresh basis for more than SEQ_MAX_GAP samples.
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
const uint16_t SEQ_MAX_GAP = 64;               // samples, ~3 s at 20 Hz; while linked, gaps are < ~20

// Protocol version: byte 0 of both payloads; with the exact size it rejects a
// peer running another format (v1, before this field, was 12 + 12 bytes).
// Both boards must run the same version.
const uint8_t PROTO_VER = 2;

// Body -> chair, one per IMU sample. Besides the sample, it reports what the
// body was actually doing when it sent it: the control it applied at its
// previous sample (= its CSV row seq-1) and the motor output right now.
enum : uint8_t {                               // BodyPacket.ctl: who decides the body's alert
  CTL_NONE     = 0,                            // no valid ChairAck since boot: no reference, no alert
  CTL_CHAIR    = 1,                            // link up: the chair's CF_ALERT is applied
  CTL_FALLBACK = 2,                            // link lost: local rule, with the reference held
};
enum : uint8_t {                               // BodyPacket.flags
  BF_CAL   = 1 << 0,                           // holds a chair reference (calId)
  BF_ALERT = 1 << 1,                           // alert in effect (from the chair or local, see ctl)
  BF_MOTOR = 1 << 2,                           // motor/LED output commanded on now (not measured)
  BF_TEST  = 1 << 3,                           // a motor-test buzz is running (not a posture alert)
};
const uint8_t AGE_MAX = 254;                   // BodyPacket.age: 254 = 254 samples or more
const uint8_t AGE_NONE = 255;                  //   unknown: no ChairAck held, or not based on a
                                               //   sample of this body session
struct __attribute__((packed)) BodyPacket {
  uint8_t  ver;                                // PROTO_VER
  uint8_t  boot;                               // random 1..255 per body boot (never 0)
  uint16_t seq;                                // +1 per sample (wraps 65535 -> 0)
  float    pitch, roll;                        // degrees, from the accelerometer
  uint8_t  ctl;                                // CTL_*
  uint8_t  flags;                              // BF_*
  uint8_t  calId;                              // ChairAck.calId of the ChairAck held; 0 = none
  uint8_t  age;                                // seq - ackSeq (samples); AGE_MAX, AGE_NONE
  uint16_t ackSeq;                             // basisSeq of the ChairAck held (if age != AGE_NONE)
};

// Chair -> body, as the ESB ACK payload. Preloaded after the chair decides on
// body sample N, so it rides on the ACK of N+1 (or of a retry of N): the body
// gets decisions one sample late. basis* says which sample it was made on.
enum : uint8_t {                               // ChairAck.flags
  CF_CAL   = 1 << 0,                           // reference valid (pitchRef/rollRef)
  CF_ALERT = 1 << 1,                           // chair says: alert now
};
struct __attribute__((packed)) ChairAck {
  uint8_t  ver;                                // PROTO_VER
  uint8_t  flags;                              // CF_*
  uint8_t  buzz;                               // +1 per motor-test request (body buzzes on change)
  uint8_t  calId;                              // reference id: a new one on every capture and
                                               // clear; random start per chair boot; never 0
  float    pitchRef, rollRef;                  // calibrated upright, for the body's fallback
  uint16_t basisSeq;                           // body sample this decision was made on...
  uint8_t  basisBoot;                          // ...and its boot id; 0 = not made on a sample
                                               // (chair boot, body link timeout, calibration cleared)
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

// What anglesFromAccel() (atan2) can produce: pitch in [-90, 90], roll in
// [-180, 180]; references are means of such samples. False for NaN too.
inline bool anglesOk(float pitch, float roll) {
  return pitch >= -90.0f && pitch <= 90.0f && roll >= -180.0f && roll <= 180.0f;
}

// A ChairAck is only trusted if it has the exact size and sane, coherent
// contents (no alert without a reference, no seq without a basis).
inline bool parseChairAck(const void *buf, uint8_t len, ChairAck &out) {
  if (len != sizeof(ChairAck)) return false;
  ChairAck a;
  memcpy(&a, buf, sizeof(a));
  if (a.ver != PROTO_VER || a.calId == 0) return false;
  if (a.flags & ~(CF_CAL | CF_ALERT)) return false;
  if ((a.flags & CF_ALERT) && !(a.flags & CF_CAL)) return false;
  if ((a.flags & CF_CAL) && !anglesOk(a.pitchRef, a.rollRef)) return false;
  if (!a.basisBoot && a.basisSeq) return false;
  out = a;
  return true;
}

// Same for the chair: a packet of another format, or whose report doesn't add
// up, is rejected before it can count as a sample (or keep the link up).
inline bool parseBodyPacket(const void *buf, uint8_t len, BodyPacket &out) {
  if (len != sizeof(BodyPacket)) return false;
  BodyPacket p;
  memcpy(&p, buf, sizeof(p));
  if (p.ver != PROTO_VER || p.boot == 0) return false;
  if (!anglesOk(p.pitch, p.roll)) return false;
  if (p.ctl > CTL_FALLBACK) return false;
  if (p.flags & ~(BF_CAL | BF_ALERT | BF_MOTOR | BF_TEST)) return false;
  if ((p.flags & BF_ALERT) && !(p.flags & BF_CAL)) return false;
  if (p.ctl == CTL_NONE && (p.calId || (p.flags & BF_CAL) || p.age != AGE_NONE)) return false;
  if (p.ctl != CTL_NONE && !p.calId) return false;
  if (p.age == AGE_NONE ? p.ackSeq != 0
                        : p.age < AGE_MAX && (uint16_t)(p.seq - p.ackSeq) != p.age) return false;
  out = p;
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
//
// Age of the ChairAck held, in this body's own samples (never another board's
// millis()). n = samples sent since boot, the u16 seq is its low half. A
// basis is accepted once, when it first arrives: it must be a sample of this
// body session (basisBoot == boot) at most SEQ_MAX_GAP behind (u16 delta) —
// the chair drops its basis 1 s after its last sample, so anything older is
// another session reusing the boot id. From then on the age only grows with
// n (u32): repeated ACKs with the same basis don't reset it, and the seq
// wrapping can't make an old decision look new (nor a rejected basis pass
// later). A ChairAck with no basis, or one that fails the check, is still
// applied (control is not judged by its basis) but its age is unknown: the
// previous basis no longer describes the state held.
struct BodyLink {
  ChairAck ack = {};
  bool haveAck = false;                        // ever got a valid ChairAck
  uint8_t lastBuzz = 0;
  LinkWatch watch;
  bool basisOk = false;                        // ack.basis* is a sample of this session...
  uint32_t basisN = 0;                         // ...namely sample number basisN
  // since clearCounts(); noack = sent - acked; acked = ctl + bad
  uint32_t sent = 0, acked = 0, retx = 0, ctl = 0, bad = 0;

  struct Result { bool ctl, buzz, linkChanged; };

  // n/boot: this write's sample number and the body's boot id.
  // ok/arc: radio.write() result and getARC() (retransmissions of this write,
  // also when it ran out of retries). payload/len: the ACK payload (len 0 if none).
  Result onTx(uint32_t t, uint32_t n, uint8_t boot, bool ok, uint8_t arc,
              const void *payload, uint8_t len) {
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
        takeBasis(a, n, boot);
        ack = a;
        haveAck = true;
      } else {
        bad++;
      }
    }
    r.linkChanged = watch.update(t, r.ctl);
    return r;
  }

  // Who decides at sample n (after its ACK, if any): CTL_*.
  uint8_t control() const { return watch.up ? CTL_CHAIR : haveAck ? CTL_FALLBACK : CTL_NONE; }

  // Samples from the held ChairAck's basis to sample n; AGE_MAX = that many or
  // more, AGE_NONE = unknown.
  uint8_t age(uint32_t n) const {
    if (!haveAck || !basisOk) return AGE_NONE;
    uint32_t d = n - basisN;
    return d < AGE_MAX ? (uint8_t)d : AGE_MAX;
  }

  void clearCounts() { sent = acked = retx = ctl = bad = 0; }

 private:
  void takeBasis(const ChairAck &a, uint32_t n, uint8_t boot) {
    // The same basis again: judged (and counted) at its first arrival, kept.
    if (haveAck && a.basisBoot == ack.basisBoot && a.basisSeq == ack.basisSeq) return;
    uint16_t d = (uint16_t)n - a.basisSeq;
    basisOk = a.basisBoot && a.basisBoot == boot && d <= SEQ_MAX_GAP && d <= n;
    basisN = basisOk ? n - d : 0;
  }
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

// lost = seq numbers skipped between two packets of the same session received
// less than LINK_TIMEOUT_MS apart. Packets missed during an outage are not
// counted (the outage itself is the "# body link lost/up" events), nor across a
// restart. Wrap 65535 -> 0 is an ordinary step.
// Reboots, best effort: the boot id is 1 random byte, so a reboot keeps the
// same id with probability 1/255. Then it is inferred from seq: back after an outage -> RESTART (the
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
