/*
 * XIAO nRF52840 Sense (body unit) — IMU posture alert + chair ESB receiver
 * -----------------------------------------------------------------------
 * The "brain". Samples the onboard IMU at 20 Hz, turns the gravity vector into
 * pitch/roll, and alerts when the deviation from a calibrated "upright"
 * reference stays above ALERT_DEG for ALERT_HOLD_MS. The chair's FSR packets
 * (firmware/chair, over ESB) are received and logged; stage 4 fuses them.
 *
 * Libraries: nrf_to_nrf (TMRh20, ESB; runs the RADIO directly, no SoftDevice)
 * and Seeed Arduino LSM6DS3. nrf_to_nrf is polled: the ACK is only sent from
 * inside radio.available(), so loop() must never block (the chair retries for
 * ~20 ms before giving up) — hence millis()-scheduled work, no delay().
 *
 * Calibration: sit upright and press the chair's button (CMD_CALIBRATE over
 * ESB; for now the chair's serial 'c'), or send 'c' here. The reference is the
 * mean of the next CAL_SAMPLES samples (1 s). 'r' clears it. No alerts until
 * calibrated.
 *
 * Alert output: vibration motor on D10 (NPN low-side driver + flyback diode,
 * motor fed from 3V3; PWM duty MOTOR_DUTY sets the strength), mirrored on the
 * onboard red LED. Pulsed PULSE_ON_MS every PULSE_PERIOD_MS while alerting.
 * 'm' buzzes once (MOTOR_TEST_MS) to bench-test the motor.
 *
 * Output (prose lines are '#'-prefixed; pandas read_csv(comment='#')), 20 Hz:
 *   t_ms,cal,dev,alert,led,dpitch,droll,pitch,roll,seq,c8,...,c15
 *   dev   = sqrt(dpitch^2 + droll^2), degrees from upright (0 if uncalibrated)
 *   alert = 1 while in alert (dev held above threshold), led = motor/LED on now
 *   seq,c8..c15 = last chair packet (raw ADC counts); seq=-1 before the first
 * Events: "# alert ON/OFF", "# calibrat...", once per second "# rx pkts=N lost=M".
 *
 * Build / upload / monitor (arduino-cli, run from the repo root):
 *   arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense firmware/body
 *   arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p /dev/ttyACM1 firmware/body
 *   arduino-cli monitor -p /dev/ttyACM1 -c baudrate=115200
 */

#include <Adafruit_TinyUSB.h>                  // needed for USB Serial on this core
#include <nrf_to_nrf.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>

// --- ESB link (keep in sync with firmware/chair/chair.ino) ---
const uint8_t RF_ADDR[6]  = "PCHR1";          // 5-byte pipe address
const uint8_t RF_CHANNEL  = 76;               // 2476 MHz
enum : uint8_t { CMD_NONE = 0, CMD_CALIBRATE = 1 };
struct __attribute__((packed)) ChairPacket {
  uint16_t seq;                                // +1 per scan; gaps = lost packets
  uint8_t  cmd;                                // CMD_*
  uint8_t  reserved;
  uint16_t raw[8];                             // ADC counts, C8..C15
};

// --- Posture alert ---
const float ALERT_DEG      = 10.0f;            // analysis/VALIDATION.md: upright p95 6.4°, bad p05 18.6°
const float CLEAR_DEG      = 7.0f;             // hysteresis: alert ends below this
const unsigned long ALERT_HOLD_MS = 3000;      // dev must stay > ALERT_DEG this long
const unsigned long SAMPLE_MS     = 50;        // 20 Hz
const int CAL_SAMPLES             = 20;        // 1 s of samples averaged into the reference

// Alert output: motor driver (active HIGH, PWM) + red LED (active LOW) mirror
const int MOTOR_PIN  = D10;
const int MOTOR_DUTY = 30;                     // 0..255; lower = weaker buzz
const int LED_PIN    = LED_RED;
const unsigned long PULSE_PERIOD_MS = 1000, PULSE_ON_MS = 300;
const unsigned long MOTOR_TEST_MS   = 500;

const float RAD_TO_DEGF = 57.2957795f;

nrf_to_nrf radio;
LSM6DS3 imu(I2C_MODE, 0x6A);                   // onboard LSM6DS3TR-C

// chair link
ChairPacket lastPkt;
bool haveSeq = false;
uint16_t nPkts = 0, nLost = 0;                 // reported and reset once per second
unsigned long lastReport = 0;

// calibration
bool  calibrated = false;
int   calLeft = 0;                             // >0 while capturing the reference
float calPSum = 0, calRSum = 0;
float pitchRef = 0, rollRef = 0;

// alert state
bool alert = false, ledState = false;
unsigned long overSince = 0, alertStart = 0;   // overSince = 0: not over threshold
unsigned long lastSample = 0;
unsigned long testUntil = 0;                   // 'm' bench buzz; 0 = idle

// Axis mapping for this mounting (see xiao_imu_test.ino): pitch = forward/back.
void anglesFromAccel(float aX, float aY, float aZ, float &pitch, float &roll) {
  pitch = atan2f(-aX, sqrtf(aY * aY + aZ * aZ)) * RAD_TO_DEGF;
  roll  = atan2f(aY, aZ) * RAD_TO_DEGF;
}

void startCalibration(const char *src) {
  calLeft = CAL_SAMPLES;
  calPSum = calRSum = 0;
  Serial.print("# calibrating (hold upright 1 s), from "); Serial.println(src);
}

void setLed(bool on) {
  ledState = on;
  analogWrite(MOTOR_PIN, on ? MOTOR_DUTY : 0);
  digitalWrite(LED_PIN, on ? LOW : HIGH);
}

void setAlert(bool on, float dev) {
  if (on == alert) return;
  alert = on;
  alertStart = millis();
  Serial.print(on ? "# alert ON  dev=" : "# alert OFF dev="); Serial.println(dev, 1);
  if (!on) setLed(false);
}

void pollRadio() {
  if (!radio.available()) return;
  ChairPacket pkt;
  uint8_t len = radio.getDynamicPayloadSize();
  radio.read(&pkt, sizeof(pkt));
  if (len != sizeof(pkt)) return;
  if (haveSeq) nLost += (uint16_t)(pkt.seq - lastPkt.seq - 1);
  haveSeq = true;
  lastPkt = pkt;
  nPkts++;
  if (pkt.cmd == CMD_CALIBRATE) startCalibration("chair");
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') startCalibration("serial");
    else if (c == 'r' || c == 'R') {
      calibrated = false; calLeft = 0; overSince = 0;
      setAlert(false, 0);
      Serial.println("# calibration cleared");
    }
    else if (c == 'm' || c == 'M') {
      testUntil = millis() + MOTOR_TEST_MS;
      setLed(true);
      Serial.println("# motor test");
    }
  }
}

void sample(unsigned long t) {
  float pitch, roll;
  anglesFromAccel(imu.readFloatAccelX(), imu.readFloatAccelY(), imu.readFloatAccelZ(),
                  pitch, roll);

  if (calLeft > 0) {
    calPSum += pitch; calRSum += roll;
    if (--calLeft == 0) {
      pitchRef = calPSum / CAL_SAMPLES;
      rollRef  = calRSum / CAL_SAMPLES;
      calibrated = true; overSince = 0;
      setAlert(false, 0);
      Serial.print("# calibrated upright: pitch="); Serial.print(pitchRef, 1);
      Serial.print(" roll="); Serial.println(rollRef, 1);
    }
  }

  float dPitch = pitch - pitchRef, dRoll = roll - rollRef;
  float dev = calibrated ? sqrtf(dPitch * dPitch + dRoll * dRoll) : 0;

  // over threshold for ALERT_HOLD_MS -> alert; below CLEAR_DEG -> clear
  if (calibrated && calLeft == 0) {
    if (dev > ALERT_DEG) {
      if (!overSince) overSince = t ? t : 1;
      if (!alert && t - overSince >= ALERT_HOLD_MS) setAlert(true, dev);
    } else {
      overSince = 0;
      if (alert && dev < CLEAR_DEG) setAlert(false, dev);
    }
  }

  // t_ms,cal,dev,alert,led,dpitch,droll,pitch,roll,seq,c8..c15
  Serial.print(t);              Serial.print(',');
  Serial.print(calibrated);     Serial.print(',');
  Serial.print(dev, 1);         Serial.print(',');
  Serial.print(alert);          Serial.print(',');
  Serial.print(ledState);          Serial.print(',');
  Serial.print(dPitch, 1);      Serial.print(',');
  Serial.print(dRoll, 1);       Serial.print(',');
  Serial.print(pitch, 1);       Serial.print(',');
  Serial.print(roll, 1);        Serial.print(',');
  Serial.print(haveSeq ? (long)lastPkt.seq : -1L);
  for (int i = 0; i < 8; i++) { Serial.print(','); Serial.print(haveSeq ? lastPkt.raw[i] : 0); }
  Serial.println();
}

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // give the USB monitor a moment

  if (imu.begin() != 0) Serial.println("# IMU init FAILED");
  if (!radio.begin())   Serial.println("# radio init FAILED");
  radio.setChannel(RF_CHANNEL);
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, RF_ADDR);
  radio.startListening();                      // RX role

  Serial.println("# body unit: sit upright, then calibrate (chair 'c' or 'c' here); 'r' clears; 'm' buzzes");
  Serial.println("t_ms,cal,dev,alert,led,dpitch,droll,pitch,roll,seq,c8,c9,c10,c11,c12,c13,c14,c15");
}

void loop() {
  pollRadio();
  pollSerial();

  unsigned long t = millis();
  if (t - lastSample >= SAMPLE_MS) {
    lastSample = t;
    sample(t);
  }

  if (alert) setLed((t - alertStart) % PULSE_PERIOD_MS < PULSE_ON_MS);
  else if (testUntil && (long)(t - testUntil) >= 0) { testUntil = 0; setLed(false); }

  if (t - lastReport >= 1000) {
    lastReport = t;
    Serial.print("# rx pkts="); Serial.print(nPkts);
    Serial.print(" lost="); Serial.println(nLost);
    nPkts = nLost = 0;
  }
}
