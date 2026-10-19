#pragma once
#include "Arduino.h"
#define I2C_MODE 0
namespace sim { extern float ax, ay, az; }
struct LSM6DS3 {
  LSM6DS3(int, int) {}
  int begin() { return 0; }
  float readFloatAccelX() { return sim::ax; }
  float readFloatAccelY() { return sim::ay; }
  float readFloatAccelZ() { return sim::az; }
};
