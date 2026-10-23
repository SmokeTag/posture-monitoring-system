/*
 * Beacon — the chair's live state as a BLE advertisement (wireless monitor)
 * ------------------------------------------------------------------------
 * Pure logic (no Arduino calls): the payload and the ADV_NONCONN_IND PDU.
 * The radio switch is in BleAdv.h; the reader is analysis/ble_chair.py
 * (keep BEACON_VER, the layout and the scaling in sync).
 *
 * One-way and best effort: anyone nearby can read it (posture state only, no
 * identity) and there is no ACK, so losses are just skipped beacons. It is for
 * the exhibit screen and monitoring, not for data captures (USB has every
 * sample).
 *
 * PDU (BLE 1M, advertising channels 37/38/39), as the RADIO sees it in RAM:
 *   [0] header 0x42 (ADV_NONCONN_IND, TxAdd = random)  [1] length = 6 + 31
 *   [2..7]  AdvA: the chip's static random address (FICR, little endian)
 *   [8..38] AD "manufacturer specific": len 30, type 0xFF, company 0xFFFF
 *           (the SIG's id for tests/internal use), then BEACON_LEN bytes:
 *
 *   off size field
 *    0   1   ver      BEACON_VER
 *    1   1   seq      +1 per beacon (the reader counts gaps; also defeats
 *                     duplicate filtering in the host's BLE stack)
 *    2   2   t16      chair millis() & 0xFFFF at this beacon (reader unwraps)
 *    4   1   flags    BK_*
 *    5   1   bflags   body report: bits 0-1 bctl, then BB_*
 *    6   1   calId
 *    7   1   reason   int8, Fusion.h Reason; -1 = no decision
 *    8   1   imu      int8, ImuStatus; -1 = body silent
 *    9   2   dpitch   int16, 0.1 deg; INT16_MIN = nan
 *   11   2   droll    int16, 0.1 deg; INT16_MIN = nan
 *   13   2   vbat     body battery mV; 0xFFFF = body silent
 *   15  12   c8..c15  FSRs, raw 12-bit ADC, packed two per 3 bytes
 *                     (a | b << 12, little endian)
 * All multi-byte fields little endian.
 */
#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "FloatBits.h"

namespace posture {

const uint8_t  BEACON_VER = 1;
const uint8_t  BEACON_LEN = 27;                // fills the 31-byte AdvData
const uint16_t BEACON_COMPANY = 0xFFFF;
const unsigned long BEACON_MS = 50;            // silent-body timer; linked: every sample (~20 Hz)
const uint8_t  ADV_PDU_LEN = 2 + 6 + 31;       // header + length + AdvA + AdvData

enum : uint8_t {                               // Beacon flags
  BK_CAL     = 1 << 0,                         // reference valid
  BK_ALERT   = 1 << 1,                         // chair's alert decision
  BK_BODY    = 1 << 2,                         // body link up (else body fields void)
  BK_CAPTURE = 1 << 3,                         // capturing the reference (1 s)
  BK_MUTE    = 1 << 4,                         // motor muted
  BK_RADIO   = 1 << 5,                         // radio init failed
  BK_REJECT  = 1 << 6,                         // last capture rejected (shown for 2 s)
  BK_USB     = 1 << 7,                         // a USB host has the serial port open
};
enum : uint8_t {                               // Beacon bflags, above the 2-bit bctl
  BB_ALERT = 1 << 2, BB_MOTOR = 1 << 3, BB_TEST = 1 << 4, BB_INH = 1 << 5,
  BB_MUTE  = 1 << 6, BB_CAL   = 1 << 7,
};

struct BeaconState {
  uint8_t seq = 0;
  uint32_t t = 0;                              // millis()
  uint8_t flags = 0, bflags = 0, calId = 0;
  int8_t reason = -1, imu = -1;
  float dPitch = NAN, dRoll = NAN;
  uint16_t vbat = 0xFFFF;
  uint16_t fsr[8] = {};
};

inline int16_t beaconAngle(float deg) {
  if (!floatIsFinite(deg)) return INT16_MIN;
  float v = deg * 10.0f;
  if (v > 32767.0f) return 32767;
  if (v < -32767.0f) return -32767;
  return (int16_t)(v < 0 ? v - 0.5f : v + 0.5f);
}

inline void beaconPayload(const BeaconState &s, uint8_t out[BEACON_LEN]) {
  int16_t dp = beaconAngle(s.dPitch), dr = beaconAngle(s.dRoll);
  out[0] = BEACON_VER;
  out[1] = s.seq;
  out[2] = (uint8_t)s.t; out[3] = (uint8_t)(s.t >> 8);
  out[4] = s.flags; out[5] = s.bflags; out[6] = s.calId;
  out[7] = (uint8_t)s.reason; out[8] = (uint8_t)s.imu;
  out[9]  = (uint8_t)dp; out[10] = (uint8_t)((uint16_t)dp >> 8);
  out[11] = (uint8_t)dr; out[12] = (uint8_t)((uint16_t)dr >> 8);
  out[13] = (uint8_t)s.vbat; out[14] = (uint8_t)(s.vbat >> 8);
  for (int i = 0; i < 4; i++) {
    uint32_t v = (s.fsr[2 * i] & 0xFFFu) | (uint32_t)(s.fsr[2 * i + 1] & 0xFFFu) << 12;
    out[15 + 3 * i] = (uint8_t)v;
    out[16 + 3 * i] = (uint8_t)(v >> 8);
    out[17 + 3 * i] = (uint8_t)(v >> 16);
  }
}

// The whole PDU as the RADIO sends it (S0 = header, LENGTH, payload).
// addr = the 48-bit device address, low byte first.
inline void advPdu(const uint8_t addr[6], const BeaconState &s, uint8_t pdu[ADV_PDU_LEN]) {
  pdu[0] = 0x42;                               // ADV_NONCONN_IND, TxAdd = random
  pdu[1] = 6 + 31;
  memcpy(pdu + 2, addr, 6);
  pdu[8] = 1 + 2 + BEACON_LEN;                 // AD length (type + company + data)
  pdu[9] = 0xFF;                               // manufacturer specific data
  pdu[10] = (uint8_t)BEACON_COMPANY; pdu[11] = (uint8_t)(BEACON_COMPANY >> 8);
  beaconPayload(s, pdu + 12);
}

}  // namespace posture
