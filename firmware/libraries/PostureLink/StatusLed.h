/*
 * StatusLed — the XIAO's onboard RGB LED as a system status light
 * ---------------------------------------------------------------
 * Same colours on both boards (each picks its own state in loop()):
 *   orange          no link (body: deciding alone if it has a reference)
 *   blue            linked, not calibrated
 *   green           linked and calibrated
 *   green, fast     chair only: capturing the reference (1 s)
 *   red, slow       posture alert (body: in step with the motor)
 *   red, fast       fault: radio init failed / IMU not delivering
 *   magenta, 3 x    body only: battery low, powering off (System OFF)
 *
 * The LED is active LOW and driven by PWM (analogWrite, 8 bit), so orange is
 * a red/green mix and `level` dims the whole LED (the body runs on a LiPo).
 * analogWrite restarts the PWM sequence even for the same duty: a channel is
 * written only when it changes, so calling set() every loop() is free.
 */
#pragma once
#include <Arduino.h>
#include <stdint.h>

namespace posture {

struct Rgb { uint8_t r, g, b; };

const Rgb RGB_OFF    = {0, 0, 0};
const Rgb RGB_RED    = {255, 0, 0};
const Rgb RGB_GREEN  = {0, 255, 0};
const Rgb RGB_BLUE   = {0, 0, 255};
const Rgb RGB_ORANGE = {255, 165, 0};          // tuned by eye: {255, 40/100, 0} looked red
const Rgb RGB_MAGENTA = {255, 0, 255};

const unsigned long BLINK_FAST_MS = 100;       // fault / capturing: 100 ms on, 100 off

// On for the first onMs of every periodMs since `since` (blink/pulse patterns).
inline bool phaseOn(unsigned long t, unsigned long since, unsigned long periodMs,
                    unsigned long onMs) {
  return (t - since) % periodMs < onMs;
}

class StatusLed {
 public:
  StatusLed(int r, int g, int b, uint8_t level) : pins_{r, g, b}, level_(level) {}

  void begin() {
    for (int i = 0; i < 3; i++) { pinMode(pins_[i], OUTPUT); write(i, 0); }
  }

  void set(Rgb c) {
    const uint8_t v[3] = {c.r, c.g, c.b};
    for (int i = 0; i < 3; i++) {
      uint8_t duty = (uint8_t)((v[i] * level_ + 127) / 255);
      if (duty != duty_[i]) write(i, duty);
    }
  }

  // Off for good (before System OFF, where PWM stops and each pin falls back
  // to its GPIO output latch): latch HIGH = off for this active-LOW LED.
  void shutdown() {
    set(RGB_OFF);
    for (int i = 0; i < 3; i++) digitalWrite(pins_[i], HIGH);
  }

  uint8_t duty(int i) const { return duty_[i]; }  // 0..255 lit fraction (tests)

 private:
  void write(int i, uint8_t duty) {
    duty_[i] = duty;
    analogWrite(pins_[i], 255 - duty);          // active LOW: 255 = off
  }

  int pins_[3];
  uint8_t level_;
  uint8_t duty_[3] = {};
};

}  // namespace posture
