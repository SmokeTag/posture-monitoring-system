/*
 * XIAO nRF52840 Sense (body unit) — IMU sensor + vibration motor, ESB transmitter
 * -------------------------------------------------------------------------------
 * Samples the onboard IMU at 20 Hz, turns the gravity vector into pitch/roll
 * and sends it to the chair unit (firmware/chair, the "brain") over Nordic ESB.
 * The chair's decision comes back in the ACK payload (CF_ALERT) and drives the
 * motor. Protocol + alert rule: firmware/libraries/PostureLink/PostureLink.h.
 *
 * Fallback: if no ACK arrives for LINK_TIMEOUT_MS, the body applies the same
 * alert rule by itself, using the last reference the chair sent (no alerts if
 * it never had one). When the link returns, the chair is in charge again.
 *
 * Libraries: nrf_to_nrf (TMRh20, ESB; runs the RADIO directly, no SoftDevice),
 * Seeed Arduino LSM6DS3, PostureLink (firmware/libraries). As the transmitter,
 * radio.write() blocks until ACKed or out of retries (~20 ms) — fine here.
 *
 * Alert output: vibration motor on D10 (NPN low-side driver + flyback diode,
 * motor fed from 3V3; PWM duty MOTOR_DUTY sets the strength), mirrored on the
 * onboard red LED. Pulsed PULSE_ON_MS every PULSE_PERIOD_MS while alerting.
 * A motor test ('m' here, or 'm' on the chair) buzzes once (MOTOR_TEST_MS).
 *
 * Output (prose lines are '#'-prefixed; pandas read_csv(comment='#')), 20 Hz:
 *   t_ms,link,cal,dev,alert,led,pitch,roll
 *   link  = 1 while the chair ACKs; 0 = fallback (body decides alone)
 *   cal   = 1 if the body holds a reference from the chair
 *   dev   = degrees from that reference (0 if none)
 *   alert = alert in effect (chair's while linked, local in fallback)
 *   led   = motor/LED on now
 * Events: "# link up/lost", "# alert ON/OFF", once per second "# tx ok=N/M retx=R".
 *
 * Build / upload / monitor (arduino-cli, run from the repo root):
 *   arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense --libraries firmware/libraries firmware/body
 *   arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p /dev/ttyACM1 firmware/body
 *   arduino-cli monitor -p /dev/ttyACM1 -c baudrate=115200
 */

#include <Adafruit_TinyUSB.h>                  // needed for USB Serial on this core
#include <nrf_to_nrf.h>
#include <PostureLink.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>

using namespace posture;

const unsigned long SAMPLE_MS = 50;            // 20 Hz

// Alert output: motor driver (active HIGH, PWM) + red LED (active LOW) mirror
const int MOTOR_PIN  = D10;
const int MOTOR_DUTY = 200;                    // 0..255; lower = weaker buzz
const int LED_PIN    = LED_RED;
const unsigned long PULSE_PERIOD_MS = 1000, PULSE_ON_MS = 300;
const unsigned long MOTOR_TEST_MS   = 500;

const float RAD_TO_DEGF = 57.2957795f;

nrf_to_nrf radio;
LSM6DS3 imu(I2C_MODE, 0x6A);                   // onboard LSM6DS3TR-C

// chair link
uint16_t seq = 0;
ChairAck lastAck = {};                         // CF_CAL/refs survive link loss (fallback)
bool haveAck = false, linkUp = false;
unsigned long lastAckAt = 0;
uint8_t lastBuzz = 0;
uint16_t nSent = 0, nOk = 0, nRetx = 0;        // reported and reset once per second
unsigned long lastReport = 0;

// alert state
AlertGate localGate;                           // only used in fallback
bool alert = false, ledState = false;
unsigned long alertStart = 0;
unsigned long lastSample = 0;
unsigned long testUntil = 0;                   // motor test buzz; 0 = idle

// Axis mapping for this mounting (see archive/xiao_imu_test): pitch = forward/back.
void anglesFromAccel(float aX, float aY, float aZ, float &pitch, float &roll) {
  pitch = atan2f(-aX, sqrtf(aY * aY + aZ * aZ)) * RAD_TO_DEGF;
  roll  = atan2f(aY, aZ) * RAD_TO_DEGF;
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

void motorTest(const char *src) {
  testUntil = millis() + MOTOR_TEST_MS;
  setLed(true);
  Serial.print("# motor test ("); Serial.print(src); Serial.println(")");
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'm' || c == 'M') motorTest("serial");
  }
}

// Send one sample; returns true if the chair ACKed it (lastAck then updated).
bool sendSample(float pitch, float roll) {
  BodyPacket pkt;
  pkt.seq = seq++;
  pkt.flags = ledState ? BF_MOTOR : 0;
  pkt.reserved = 0;
  pkt.pitch = pitch;
  pkt.roll = roll;

  bool ok = radio.write(&pkt, sizeof(pkt));
  nSent++;
  if (!ok) return false;
  nOk++;
  nRetx += radio.getARC();

  if (radio.available()) {
    ChairAck ack;
    uint8_t len = radio.getDynamicPayloadSize();
    radio.read(&ack, sizeof(ack));
    if (len == sizeof(ack)) {
      // a fresh link (boot, chair reset) only syncs the buzz counter
      if (linkUp && ack.buzz != lastBuzz) motorTest("chair");
      lastBuzz = ack.buzz;
      lastAck = ack;
      haveAck = true;
    }
  }
  return true;
}

void sample(unsigned long t) {
  float pitch, roll;
  anglesFromAccel(imu.readFloatAccelX(), imu.readFloatAccelY(), imu.readFloatAccelZ(),
                  pitch, roll);

  if (sendSample(pitch, roll) && haveAck) lastAckAt = t;
  bool up = haveAck && t - lastAckAt < LINK_TIMEOUT_MS;
  if (up != linkUp) {
    linkUp = up;
    localGate.reset();
    Serial.println(up ? "# link up (chair decides)" : "# link lost: fallback, IMU only");
  }

  bool cal = lastAck.flags & CF_CAL;
  float dev = cal ? deviation(pitch, roll, lastAck.pitchRef, lastAck.rollRef) : 0;

  if (linkUp)   setAlert(lastAck.flags & CF_ALERT, dev);
  else if (cal) { localGate.update(dev, t); setAlert(localGate.alert, dev); }
  else          setAlert(false, dev);

  // t_ms,link,cal,dev,alert,led,pitch,roll
  Serial.print(t);          Serial.print(',');
  Serial.print(linkUp);     Serial.print(',');
  Serial.print(cal);        Serial.print(',');
  Serial.print(dev, 1);     Serial.print(',');
  Serial.print(alert);      Serial.print(',');
  Serial.print(ledState);   Serial.print(',');
  Serial.print(pitch, 1);   Serial.print(',');
  Serial.println(roll, 1);
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
  radio.enableAckPayload();
  radio.setRetries(5, 15);                     // up to 15 retries, ~1.3 ms apart
  radio.openWritingPipe(RF_ADDR);
  radio.stopListening();                       // TX role

  Serial.println("# body unit: IMU -> chair (the brain). Calibrate on the chair; 'm' buzzes");
  Serial.println("t_ms,link,cal,dev,alert,led,pitch,roll");
}

void loop() {
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
    Serial.print("# tx ok="); Serial.print(nOk); Serial.print('/'); Serial.print(nSent);
    Serial.print(" retx="); Serial.println(nRetx);
    nSent = nOk = nRetx = 0;
  }
}
