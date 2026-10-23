// Host stand-in for PostureLink/BleAdv.h: records every advertised PDU.
#pragma once
#include "Arduino.h"
#include "nrf_to_nrf.h"
#include <vector>

namespace sim {
inline std::vector<std::vector<uint8_t>> &bleSent() { static std::vector<std::vector<uint8_t>> v; return v; }
inline int &blePendingSends() { static int n = 0; return n; }   // sent while an ESB packet was pending
inline FakeRadio *&bleRadio() { static FakeRadio *r = nullptr; return r; }
}

namespace posture {

inline void bleAddress(uint8_t a[6]) {
  const uint8_t addr[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0xC6};
  memcpy(a, addr, 6);
}

inline bool bleAdvertise(const uint8_t *pdu) {
  if (sim::bleRadio() && !sim::bleRadio()->rx.empty()) sim::blePendingSends()++;
  sim::bleSent().emplace_back(pdu, pdu + 2 + pdu[1]);
  return true;
}

}  // namespace posture
