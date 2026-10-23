/*
 * BleAdv — one BLE advertising event from a RADIO that runs ESB (nrf_to_nrf)
 * -------------------------------------------------------------------------
 * firmware/chair only; payload in Beacon.h. nrf_to_nrf drives the RADIO
 * directly (no SoftDevice, polled, no interrupts), so between two of its
 * calls the RADIO is ours: save the registers it set, switch to BLE 1M, send
 * the PDU once on channels 37, 38 and 39 (~1.3 ms in all), put every register
 * back. The caller then restarts RX (radio.startListening()) and must only
 * call this when no ESB packet is pending (right after one was read): a body
 * packet that arrives meanwhile is missed and retried by the body (~20 ms).
 * The 16 MHz crystal must be running (chair.ino keepCrystal()).
 *
 * The host tests shadow this header with test/stub/BleAdv.h.
 */
#pragma once
#include <Arduino.h>

namespace posture {

// The chip's static random address (FICR), low byte first; top 2 bits = 11.
inline void bleAddress(uint8_t a[6]) {
  uint32_t lo = NRF_FICR->DEVICEADDR[0], hi = NRF_FICR->DEVICEADDR[1];
  for (int i = 0; i < 4; i++) a[i] = (uint8_t)(lo >> (8 * i));
  a[4] = (uint8_t)hi;
  a[5] = (uint8_t)(hi >> 8) | 0xC0;
}

// Bounded wait (~2 ms at 64 MHz): a stuck RADIO must not hang loop().
inline bool bleWait(volatile uint32_t &ev) {
  for (uint32_t n = 0; n < 40000; n++) if (ev) return true;
  return false;
}

inline bool bleDisable() {
  NRF_RADIO->SHORTS = 0;
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  bool ok = bleWait(NRF_RADIO->EVENTS_DISABLED);
  NRF_RADIO->EVENTS_DISABLED = 0;
  return ok;
}

// Sends pdu (header, length, payload; ADV_PDU_LEN bytes in Beacon.h) on the
// three advertising channels. False if the RADIO didn't respond in time
// (registers restored anyway).
inline bool bleAdvertise(const uint8_t *pdu) {
  static uint8_t buf[40];                      // the RADIO reads it via EasyDMA: must be RAM
  memcpy(buf, pdu, 2 + pdu[1]);

  struct { uint32_t mode, pcnf0, pcnf1, base0, prefix0, txaddr, rxaddrs, crccnf, crcinit, crcpoly,
                    packetptr, freq, whiteiv, shorts, modecnf0, tifs; } s = {
    NRF_RADIO->MODE, NRF_RADIO->PCNF0, NRF_RADIO->PCNF1, NRF_RADIO->BASE0, NRF_RADIO->PREFIX0,
    NRF_RADIO->TXADDRESS, NRF_RADIO->RXADDRESSES, NRF_RADIO->CRCCNF, NRF_RADIO->CRCINIT,
    NRF_RADIO->CRCPOLY, NRF_RADIO->PACKETPTR, NRF_RADIO->FREQUENCY, NRF_RADIO->DATAWHITEIV,
    NRF_RADIO->SHORTS, NRF_RADIO->MODECNF0, NRF_RADIO->TIFS };

  bool ok = bleDisable();
  NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos;
  NRF_RADIO->PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) | (8 << RADIO_PCNF0_LFLEN_Pos) |
                     (0 << RADIO_PCNF0_S1LEN_Pos) | (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);
  NRF_RADIO->PCNF1 = (37 << RADIO_PCNF1_MAXLEN_Pos) | (0 << RADIO_PCNF1_STATLEN_Pos) |
                     (3 << RADIO_PCNF1_BALEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
                     (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);
  NRF_RADIO->BASE0 = 0x89BED600;               // advertising access address 0x8E89BED6
  NRF_RADIO->PREFIX0 = (s.prefix0 & ~0xFFu) | 0x8E;
  NRF_RADIO->TXADDRESS = 0;
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                      (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
  NRF_RADIO->CRCPOLY = 0x00065B;
  NRF_RADIO->CRCINIT = 0x555555;
  NRF_RADIO->PACKETPTR = (uint32_t)buf;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;

  static const uint8_t CH[3] = {37, 38, 39}, FREQ[3] = {2, 26, 80};   // 2402/2426/2480 MHz
  for (int i = 0; ok && i < 3; i++) {
    NRF_RADIO->FREQUENCY = FREQ[i];
    NRF_RADIO->DATAWHITEIV = CH[i];
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_TXEN = 1;                 // ramp-up -> START -> END -> DISABLE
    ok = bleWait(NRF_RADIO->EVENTS_DISABLED);
  }
  if (!ok) bleDisable();

  NRF_RADIO->SHORTS = 0;
  NRF_RADIO->MODE = s.mode;
  NRF_RADIO->PCNF0 = s.pcnf0;
  NRF_RADIO->PCNF1 = s.pcnf1;
  NRF_RADIO->BASE0 = s.base0;
  NRF_RADIO->PREFIX0 = s.prefix0;
  NRF_RADIO->TXADDRESS = s.txaddr;
  NRF_RADIO->RXADDRESSES = s.rxaddrs;
  NRF_RADIO->CRCCNF = s.crccnf;
  NRF_RADIO->CRCINIT = s.crcinit;
  NRF_RADIO->CRCPOLY = s.crcpoly;
  NRF_RADIO->PACKETPTR = s.packetptr;
  NRF_RADIO->FREQUENCY = s.freq;
  NRF_RADIO->DATAWHITEIV = s.whiteiv;
  NRF_RADIO->MODECNF0 = s.modecnf0;
  NRF_RADIO->TIFS = s.tifs;
  NRF_RADIO->SHORTS = s.shorts;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->EVENTS_ADDRESS = 0;
  NRF_RADIO->EVENTS_PAYLOAD = 0;
  NRF_RADIO->EVENTS_CRCOK = 0;
  NRF_RADIO->EVENTS_CRCERROR = 0;
  return ok;
}

}  // namespace posture
