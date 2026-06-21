/*
 * XIAO nRF52840 Sense — IMU bring-up + posture logging
 * ----------------------------------------------------
 * Purpose: confirm the board is alive and that the onboard 6-axis IMU
 *          (LSM6DS3TR-C) can be read. This is the upper-back orientation
 *          sensor for the posture system.
 *
 * Prints human-readable tilt ANGLES (pitch + roll) derived from the
 * accelerometer's gravity vector — much easier to read as posture than the
 * raw aX/aY/aZ. For an upper-back sensor:
 *   - pitch = forward/back lean  (the slouch axis — leaning over a desk)
 *   - roll  = left/right lean
 * Angles come from gravity only, so they're absolute (no drift) but jittery
 * while you're actually moving. Raw accel/gyro trail behind for reference.
 * NOTE: the pitch/roll axis mapping assumes a particular board mounting; if
 * forward lean shows up on the wrong axis, swap the formulas below to match
 * how the XIAO ends up sitting on the back.
 *
 * Calibration: the XIAO has no usable user button (only RESET), so sit
 * upright and send 'c' over serial to capture the current pose as the
 * "upright" reference. After that the sketch reports how far pitch/roll
 * deviate from upright (dPitch/dRoll) — that deviation is the posture signal.
 * Send 'r' to clear and go back to absolute angles.
 *
 * --- CSV LOGGING (for the midterm measurement campaign) ---------------------
 * Send 'l' to toggle CSV mode. In CSV mode the sketch stops printing prose
 * and emits one comma-separated row per sample, after a single header line:
 *
 *   t_ms,tag,cal,pitch,roll,dpitch,droll,ax,ay,az,gx,gy,gz
 *
 * Every prose/diagnostic line is prefixed with '#', so a captured file is
 * directly parseable (pandas: read_csv(comment='#'); the header is the first
 * non-# line).
 *
 * Logging is CONTINUOUS at 20 Hz once CSV mode is on — it never stops on its
 * own, and the `tag` column simply holds the last digit pressed until you
 * change it. You time each posture yourself (~10 s); there is no per-tag timer.
 *
 * Send a digit '0'..'9' to stamp `tag` and label postures/trials in ONE
 * continuous capture. Meanings:
 *   0 = upright   1 = forward slouch   2 = lean L   3 = lean R   4 = head-down
 *   5 = laid-back (hips forward, torso leaning back on the chair)
 *   9 = JUNK / MOVING  (reserved — see below)
 * Pitch convention (this mounting): upright ~45 deg; leaning FORWARD lowers
 * pitch, leaning BACK raises it (laid-back = the highest pitch).
 * (For the re-don experiment, reuse 0 and re-send 'c' before each trial.)
 * `cal` is 1 once calibrated; when not calibrated, dpitch/droll equal pitch/roll.
 *
 * EXCLUDE TRANSITIONS — tag 9. Moving between postures contaminates data two
 * ways: (1) the angle sweeps through intermediate values that belong to no
 * posture, and (2) the pitch/roll math assumes the accelerometer sees only
 * gravity, but while moving it also picks up the motion's own acceleration, so
 * those samples are briefly wrong. Convention: press 9 during every transition
 * and DROP all tag==9 rows in analysis; what remains is clean steady segments.
 * Tag 9 is timing-forgiving — roughly covering each move is enough. (Optional
 * safety margin: also drop the first ~0.5-1 s after each tag change before
 * averaging, in case a key was pressed a hair before you fully settled.)
 *
 * Capture workflow (see also docs/protocolo-captura-imu.md):
 *   arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee postures.csv
 *   # type (each line sends on Enter):
 *   #   l                -> start CSV
 *   #   c                -> calibrate while sitting upright
 *   #   0 (hold ~10s)    -> upright
 *   #   9                -> move
 *   #   1 (settle ~10s)  -> forward slouch
 *   #   9 -> 2 lean L -> 9 -> 3 lean R -> 9 -> 4 head-down   (9 on every move)
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

// Sample period: slow in live (human) mode so the scroll is readable; faster
// in CSV mode for smoother plots (still trivial data volume).
const int SAMPLE_MS_LIVE = 200;   //  5 Hz
const int SAMPLE_MS_CSV  = 50;    // 20 Hz

// --- Upright calibration state ----------------------------------------------
// Captured when the user sends 'c' over serial while sitting straight.
bool  calibrated = false;
float pitchRef   = 0.0f;
float rollRef    = 0.0f;

// --- Logging state ----------------------------------------------------------
bool csvMode = false;   // 'l' toggles CSV output for file capture
int  tag     = 0;       // '0'..'9' stamps the tag column (posture/trial label)

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
  Serial.print("# calibrated upright: pitch=");
  Serial.print(pitchRef, 1);
  Serial.print(" deg  roll=");
  Serial.print(rollRef, 1);
  Serial.println(" deg.  Reporting deviation from upright.");
}

void printCsvHeader() {
  Serial.println("t_ms,tag,cal,pitch,roll,dpitch,droll,ax,ay,az,gx,gy,gz");
}

// One-character serial commands: 'c' calibrate, 'r' clear, 'l' toggle CSV,
// '0'..'9' set the tag column. Line endings ('\n'/'\r') fall through harmlessly.
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      captureReference();
    } else if (c == 'r' || c == 'R') {
      calibrated = false;
      Serial.println("# calibration cleared — reporting absolute angles.");
    } else if (c == 'l' || c == 'L') {
      csvMode = !csvMode;
      if (csvMode) {
        Serial.println("# CSV mode ON. Header follows; '0'-'9' sets tag.");
        printCsvHeader();
      } else {
        Serial.println("# CSV mode OFF.");
      }
    } else if (c >= '0' && c <= '9') {
      tag = c - '0';
      Serial.print("# tag=");
      Serial.println(tag);
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
    Serial.println("# IMU init FAILED — check board variant is 'Sense'.");
  } else {
    Serial.println("# IMU init OK. Streaming tilt angles (deg) + raw accel/gyro.");
    Serial.println("# Sit upright and send 'c' to calibrate; 'r' clears; 'l' = CSV.");
  }
}

void loop() {
  handleSerialCommands();   // check for c/r/l/digit before reading + printing

  // Read gravity vector + angular rate. Accel units (g vs m/s^2) cancel in the
  // angle math, so it doesn't matter that accel is in g.
  float aX = imu.readFloatAccelX();
  float aY = imu.readFloatAccelY();
  float aZ = imu.readFloatAccelZ();
  float gX = imu.readFloatGyroX();
  float gY = imu.readFloatGyroY();
  float gZ = imu.readFloatGyroZ();

  float pitch, roll;
  anglesFromAccel(aX, aY, aZ, pitch, roll);
  float dPitch = pitch - pitchRef;   // pitchRef/rollRef are 0 until calibrated,
  float dRoll  = roll  - rollRef;    // so deviation == absolute angle until then

  if (csvMode) {
    // t_ms,tag,cal,pitch,roll,dpitch,droll,ax,ay,az,gx,gy,gz
    Serial.print(millis());        Serial.print(',');
    Serial.print(tag);             Serial.print(',');
    Serial.print(calibrated ? 1 : 0); Serial.print(',');
    Serial.print(pitch, 2);        Serial.print(',');
    Serial.print(roll, 2);         Serial.print(',');
    Serial.print(dPitch, 2);       Serial.print(',');
    Serial.print(dRoll, 2);        Serial.print(',');
    Serial.print(aX, 3);           Serial.print(',');
    Serial.print(aY, 3);           Serial.print(',');
    Serial.print(aZ, 3);           Serial.print(',');
    Serial.print(gX, 1);           Serial.print(',');
    Serial.print(gY, 1);           Serial.print(',');
    Serial.println(gZ, 1);
    delay(SAMPLE_MS_CSV);
    return;
  }

  // --- Human-readable live view ---------------------------------------------
  // Once calibrated, the useful number is the deviation from upright; before
  // that, show absolute tilt and remind the user to calibrate.
  if (calibrated) {
    Serial.print("dPitch="); printSigned(dPitch);
    Serial.print(" deg  dRoll="); printSigned(dRoll);
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
  Serial.print("  g(d/s): "); Serial.print(gX, 1);
  Serial.print(", "); Serial.print(gY, 1);
  Serial.print(", "); Serial.println(gZ, 1);
  delay(SAMPLE_MS_LIVE);   // 5 Hz — plenty for posture; CSV mode runs faster
}
