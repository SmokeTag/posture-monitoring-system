/*
 * XIAO nRF52840 Sense (body unit) — IMU sensor + vibration motor, ESB transmitter
 * -------------------------------------------------------------------------------
 * Samples the onboard IMU at 20 Hz, turns the gravity vector into pitch/roll
 * and sends it to the chair unit (firmware/chair, the "brain") over Nordic ESB.
 * The chair's decision comes back in the ACK payload (CF_ALERT) and drives the
 * motor. Protocol + alert rule: firmware/libraries/PostureLink/PostureLink.h.
 *
 * Fallback: if no valid ChairAck arrives for LINK_TIMEOUT_MS, the body applies
 * the same alert rule by itself, using the last reference the chair sent (no
 * alerts if it never had one). An ACK that is empty or malformed counts as
 * radio transport only: it does not keep the link up. When valid ChairAcks
 * return, the chair is in charge again.
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
 *   link  = 1 while valid ChairAcks arrive; 0 = fallback (body decides alone)
 *   cal   = 1 if the body holds a reference from the chair
 *   dev   = degrees from that reference (0 if none)
 *   alert = alert in effect (chair's while linked, local in fallback)
 *   led   = motor/LED on now
 * Events: "# link up/lost", "# alert ON/OFF", "# motor test (...)".
 * Stats, normally once per second (counts since the previous stats line; dt =
 * its span in ms, > 1000 only if the USB host stalled):
 *   # tx sent=N ack=A noack=F retx=R ctl=C bad=B sdrop=S edrop=E dt=T
 *   sent  = samples transmitted (radio.write calls)
 *   ack   = writes ACKed by the radio; noack = sent - ack (all retries spent —
 *           the chair may still have received it, only its ACK lost)
 *   retx  = retransmissions over all writes, failed ones included (+15 each)
 *   ctl   = ACKs carrying a valid ChairAck (these alone keep link = 1)
 *   bad   = ACKs with no/odd payload (ack = ctl + bad)
 *   sdrop / edrop = CSV rows / '#' events dropped because the USB host was not
 *           reading (firmware/libraries/PostureLink/SerialOut.h); the header is
 *           re-sent whenever a host opens the port.
 *
 * Build / upload / monitor (arduino-cli, run from the repo root):
 *   arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense --libraries firmware/libraries firmware/body
 *   arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p /dev/ttyACM1 firmware/body
 *   arduino-cli monitor -p /dev/ttyACM1 -c baudrate=115200
 */

#include <Adafruit_TinyUSB.h>                  // needed for USB Serial on this core
#include <nrf_to_nrf.h>
#include <PostureLink.h>
#include <SerialOut.h>
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
uint8_t bootId = 0;                            // BodyPacket.boot: random per boot
BodyLink chairLink;                            // .ack: CF_CAL/refs survive link loss (fallback)
unsigned long lastReport = 0;
bool imuFailed = false, radioFailed = false;   // reported in the header

bool bodyHeader(uint8_t i, Line &l);
SerialOut<decltype(Serial)> out(Serial, bodyHeader);

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

bool bodyHeader(uint8_t i, Line &l) {
  switch (i) {
    case 0: l.str("# body unit: IMU -> chair (the brain). Calibrate on the chair; 'm' buzzes. boot=")
             .hex8(bootId); return true;
    case 1: if (imuFailed) l.str("# IMU init FAILED"); return true;
    case 2: if (radioFailed) l.str("# radio init FAILED"); return true;
    case 3: l.str("t_ms,link,cal,dev,alert,led,pitch,roll"); return true;
  }
  return false;
}

void event(const char *s) { Line l; l.str(s); out.event(l); }

// Hardware RNG (no SoftDevice here, so it is free); never 0 (= "unknown").
uint8_t makeBootId() {
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

void setAlert(bool on, float dev) {
  if (on == alert) return;
  alert = on;
  alertStart = millis();
  Line l;
  l.str(on ? "# alert ON  dev=" : "# alert OFF dev=").fix(dev, 1);
  out.event(l);
  if (!on) setLed(false);
}

void motorTest(const char *src) {
  testUntil = millis() + MOTOR_TEST_MS;
  setLed(true);
  Line l;
  l.str("# motor test (").str(src).chr(')');
  out.event(l);
}

void pollSerial() {
  for (int n = 0; n < 16 && Serial.available() > 0; n++) {   // bounded per loop()
    char c = Serial.read();
    if (c == 'm' || c == 'M') motorTest("serial");
  }
}

// Send one sample and digest the ACK (if any) into chairLink.
void sendSample(unsigned long t, float pitch, float roll) {
  BodyPacket pkt;
  pkt.seq = seq++;
  pkt.flags = ledState ? BF_MOTOR : 0;
  pkt.boot = bootId;
  pkt.pitch = pitch;
  pkt.roll = roll;

  bool ok = radio.write(&pkt, sizeof(pkt));
  uint8_t arc = radio.getARC();                // retransmissions, also when !ok
  uint8_t buf[32];                             // = the DPL max payload
  uint8_t len = 0;
  if (ok && radio.available()) {               // the payload of THIS ACK; read() clears it
    len = radio.getDynamicPayloadSize();
    if (len > sizeof(buf)) len = sizeof(buf);
    radio.read(buf, len);
  }

  BodyLink::Result r = chairLink.onTx(t, ok, arc, buf, len);
  if (r.buzz) motorTest("chair");
  if (r.linkChanged) {
    localGate.reset();
    event(chairLink.watch.up ? "# link up (chair decides)" : "# link lost: fallback, IMU only");
  }
}

void sample(unsigned long t) {
  float pitch, roll;
  anglesFromAccel(imu.readFloatAccelX(), imu.readFloatAccelY(), imu.readFloatAccelZ(),
                  pitch, roll);

  sendSample(t, pitch, roll);
  bool linkUp = chairLink.watch.up;
  const ChairAck &ack = chairLink.ack;
  bool cal = ack.flags & CF_CAL;
  float dev = cal ? deviation(pitch, roll, ack.pitchRef, ack.rollRef) : 0;

  if (linkUp)   setAlert(ack.flags & CF_ALERT, dev);
  else if (cal) { localGate.update(dev, t); setAlert(localGate.alert, dev); }
  else          setAlert(false, dev);

  // t_ms,link,cal,dev,alert,led,pitch,roll
  Line l;
  l.u32(t).chr(',').u32(linkUp).chr(',').u32(cal).chr(',').fix(dev, 1).chr(',')
   .u32(alert).chr(',').u32(ledState).chr(',').fix(pitch, 1).chr(',').fix(roll, 1);
  out.row(l);
}

void report(unsigned long t) {
  Line l;
  l.str("# tx sent=").u32(chairLink.sent).str(" ack=").u32(chairLink.acked)
   .str(" noack=").u32(chairLink.sent - chairLink.acked).str(" retx=").u32(chairLink.retx)
   .str(" ctl=").u32(chairLink.ctl).str(" bad=").u32(chairLink.bad)
   .str(" sdrop=").u32(out.rowDrops()).str(" edrop=").u32(out.eventDrops())
   .str(" dt=").u32(t - lastReport);
  if (!out.stats(l)) return;                   // host not reading: keep counting
  lastReport = t;
  chairLink.clearCounts();
  out.clearDrops();
}

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // give the USB monitor a moment

  bootId = makeBootId();
  imuFailed = imu.begin() != 0;
  radioFailed = !radio.begin();
  radio.setChannel(RF_CHANNEL);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setRetries(5, 15);                     // up to 15 retries, ~1.3 ms apart
  radio.openWritingPipe(RF_ADDR);
  radio.stopListening();                       // TX role

  out.poll();                                  // header goes out once a host is there
}

void loop() {
  out.poll();
  pollSerial();

  unsigned long t = millis();
  if (t - lastSample >= SAMPLE_MS) {
    lastSample = t;
    sample(t);
  }

  if (alert) setLed((t - alertStart) % PULSE_PERIOD_MS < PULSE_ON_MS);
  else if (testUntil && (long)(t - testUntil) >= 0) { testUntil = 0; setLed(false); }

  if (t - lastReport >= 1000) report(t);
}
