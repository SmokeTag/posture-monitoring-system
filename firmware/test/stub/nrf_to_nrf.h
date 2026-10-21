// Host stand-in for nrf_to_nrf (TMRh20): the calls the sketches make.
#pragma once
#include "Arduino.h"
#include <deque>
#include <vector>

struct FakeRadio {
  // RX (chair): packets "in the air"; available() is true while one is pending.
  std::deque<std::vector<uint8_t>> rx;
  // ACK payload the chair preloaded (writeAckPayload)
  std::vector<uint8_t> ackPayload;
  int ackWrites = 0;
  // TX (body): what the next write() returns
  bool txOk = true;
  uint8_t txArc = 0;
  std::vector<uint8_t> txAck;                  // ACK payload delivered with an ok write
  std::vector<std::vector<uint8_t>> sent;
  bool ackPending = false;
  // Linked (test_system): write() puts the packet in `peer`'s rx and returns
  // the ACK payload `peer` has preloaded at that moment — as ESB does.
  FakeRadio *peer = nullptr;
  bool upOk = true, downOk = true;             // packet reaches the peer / its ACK comes back

  bool poweredDown = false;
  bool begin() { return true; }
  void powerDown() { poweredDown = true; }
  void setChannel(uint8_t) {}
  void enableDynamicPayloads() {}
  void enableAckPayload() {}
  void setRetries(uint8_t, uint8_t) {}
  void openReadingPipe(uint8_t, const uint8_t *) {}
  void openWritingPipe(const uint8_t *) {}
  void startListening() {}
  void stopListening() {}
  bool writeAckPayload(uint8_t, const void *b, uint8_t n) {
    ackPayload.assign((const uint8_t *)b, (const uint8_t *)b + n);
    ackWrites++;
    return true;
  }
  bool available() { return ackPending || !rx.empty(); }
  uint8_t getDynamicPayloadSize() {
    if (ackPending) return (uint8_t)txAck.size();
    return rx.empty() ? 0 : (uint8_t)rx.front().size();
  }
  void read(void *b, uint8_t n) {
    if (ackPending) { memcpy(b, txAck.data(), n < txAck.size() ? n : txAck.size()); ackPending = false; return; }
    if (rx.empty()) return;
    memcpy(b, rx.front().data(), n < rx.front().size() ? n : rx.front().size());
    rx.pop_front();
  }
  bool write(const void *b, uint8_t n) {
    sent.emplace_back((const uint8_t *)b, (const uint8_t *)b + n);
    if (peer) {
      if (upOk) peer->rx.push_back(sent.back());
      if (upOk && !downOk) peer->rx.push_back(sent.back());   // a retry: the chair sees a dup
      txOk = upOk && downOk;
      txArc = txOk ? 0 : 15;
      txAck = txOk ? peer->ackPayload : std::vector<uint8_t>();
    }
    ackPending = txOk && !txAck.empty();       // empty ACK: nothing to read
    return txOk;
  }
  uint8_t getARC() { return txArc; }
};
typedef FakeRadio nrf_to_nrf;
