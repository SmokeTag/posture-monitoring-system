/*
 * XIAO nRF52840 Sense — IMU bring-up test
 * --------------------------------------
 * Purpose: confirm the board is alive and that the onboard 6-axis IMU
 *          (LSM6DS3TR-C) can be read. This is the upper-back orientation
 *          sensor for the posture system.
 *
 * Now also prints human-readable tilt ANGLES (pitch + roll) derived from
 * the accelerometer's gravity vector — much easier to read as posture than
 * the raw aX/aY/aZ. For an upper-back sensor:
 *   - pitch = forward/back lean  (the slouch axis — leaning over a desk)
 *   - roll  = left/right lean
 * Angles come from gravity only, so they're absolute (no drift) but jittery
 * while you're actually moving. Raw accel/gyro are still printed for
 * reference. NOTE: the pitch/roll axis mapping assumes a particular board
 * mounting; if forward lean shows up on the wrong axis, swap the formulas
 * below to match how the XIAO ends up sitting on the back.
 *
 * Calibration: the XIAO has no usable user button (only RESET), so sit
 * upright and send 'c' over the Serial Monitor to capture the current pose
 * as the "upright" reference. After that the sketch prints how far pitch/roll
 * deviate from upright (dPitch/dRoll) — that deviation is the actual posture
 * signal. Send 'r' to clear and go back to absolute angles.
 *
 * Setup (Arduino IDE):
 *   1. Boards Manager: install "Seeed nRF52 Boards".
 *   2. Select board: "Seeed XIAO nRF52840 Sense".
 *   3. Library Manager: install "Seeed Arduino LSM6DS3".
 *   4. Open Serial Monitor at 115200 baud.
 *
 * The onboard IMU sits on the internal I2C bus at address 0x6A.
 *
 * Build / upload / monitor (arduino-cli, run from the repo root):
 *   arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense firmware/xiao_imu_test
 *   arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p /dev/ttyACM0 firmware/xiao_imu_test
 *   arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
 */

#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>

LSM6DS3 imu(I2C_MODE, 0x6A);   // onboard LSM6DS3TR-C, I2C address 0x6A

const float RAD_TO_DEGF = 57.2957795f;   // 180 / PI

// --- Upright calibration state ----------------------------------------------
// Captured when the user sends 'c' over serial while sitting straight.
bool  calibrated = false;
float pitchRef   = 0.0f;
float rollRef    = 0.0f;

// Tilt angles from a gravity vector. Kept in one place so the axis mapping
// only ever has to be fixed here (see the NOTE in the header).
void anglesFromAccel(float aX, float aY, float aZ, float &pitch, float &roll) {
  pitch = atan2f(-aX, sqrtf(aY * aY + aZ * aZ)) * RAD_TO_DEGF;
  roll  = atan2f(aY, aZ) * RAD_TO_DEGF;
}

// Average a short burst of samples so "upright" isn't grabbed off one noisy
// reading. Call this while the user is sitting straight.
void captureReference() {
  const int N = 20;
  float pSum = 0.0f, rSum = 0.0f, p, r;
  for (int i = 0; i < N; i++) {
    float aX = imu.readFloatAccelX();
    float aY = imu.readFloatAccelY();
    float aZ = imu.readFloatAccelZ();
    anglesFromAccel(aX, aY, aZ, p, r);
    pSum += p;
    rSum += r;
    delay(10);
  }
  pitchRef   = pSum / N;
  rollRef    = rSum / N;
  calibrated = true;
  Serial.print("== Calibrated upright: pitch=");
  Serial.print(pitchRef, 1);
  Serial.print(" deg  roll=");
  Serial.print(rollRef, 1);
  Serial.println(" deg.  Now showing deviation from upright. ==");
}

// One-character serial commands: 'c' = calibrate upright, 'r' = clear.
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      captureReference();
    } else if (c == 'r' || c == 'R') {
      calibrated = false;
      Serial.println("== Calibration cleared. Showing absolute angles. ==");
    }
  }
}

// Print a value with an explicit + or - sign (easier to scan deviations).
void printSigned(float v) {
  if (v >= 0) Serial.print('+');
  Serial.print(v, 1);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }      // wait for USB serial

  if (imu.begin() != 0) {
    Serial.println("IMU init FAILED — check board variant is 'Sense'.");
  } else {
    Serial.println("IMU init OK. Streaming tilt angles (deg) + raw accel/gyro.");
    Serial.println("Sit upright and send 'c' to calibrate; send 'r' to clear.");
  }
}

void loop() {
  handleSerialCommands();   // check for 'c' / 'r' before reading + printing

  // Read the gravity vector. Units (g vs m/s^2) cancel in the angle math, so
  // it doesn't matter that these are in g.
  float aX = imu.readFloatAccelX();
  float aY = imu.readFloatAccelY();
  float aZ = imu.readFloatAccelZ();

  float pitch, roll;
  anglesFromAccel(aX, aY, aZ, pitch, roll);

  // Once calibrated, the useful number is the deviation from upright; before
  // that, show absolute tilt and remind the user to calibrate.
  if (calibrated) {
    Serial.print("dPitch="); printSigned(pitch - pitchRef);
    Serial.print(" deg  dRoll="); printSigned(roll - rollRef);
    Serial.print(" deg");
  } else {
    Serial.print("pitch="); Serial.print(pitch, 1);
    Serial.print(" deg  roll="); Serial.print(roll, 1);
    Serial.print(" deg  [send 'c' to set upright]");
  }

  // Raw values trail behind for reference.
  Serial.print("   | a(g): "); Serial.print(aX, 3);
  Serial.print(", "); Serial.print(aY, 3);
  Serial.print(", "); Serial.print(aZ, 3);
  Serial.print("  g(d/s): "); Serial.print(imu.readFloatGyroX(), 1);
  Serial.print(", "); Serial.print(imu.readFloatGyroY(), 1);
  Serial.print(", "); Serial.println(imu.readFloatGyroZ(), 1);
  delay(200);   // 5 Hz — plenty for posture; raise later if needed
}
