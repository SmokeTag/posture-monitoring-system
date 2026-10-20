/*
 * RandomId — a random 1..255 id per boot, for firmware/body (BodyPacket.boot)
 * and firmware/chair (the first ChairAck.calId). Arduino only (NRF_RNG).
 */
#pragma once
#include <Arduino.h>

namespace posture {

// Hardware RNG (no SoftDevice here, so it is free); never 0 (= "none").
inline uint8_t randomId() {
  uint8_t id = 0;
  NRF_RNG->TASKS_START = 1;
  for (int tries = 0; tries < 8 && !id; tries++) {
    NRF_RNG->EVENTS_VALRDY = 0;
    unsigned long t0 = micros();
    while (!NRF_RNG->EVENTS_VALRDY && micros() - t0 < 1000) {}
    if (NRF_RNG->EVENTS_VALRDY) id = (uint8_t)NRF_RNG->VALUE;
  }
  NRF_RNG->TASKS_STOP = 1;
  return id ? id : (uint8_t)(1 + micros() % 255);
}

}  // namespace posture
