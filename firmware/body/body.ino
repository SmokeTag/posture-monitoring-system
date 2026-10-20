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
 * return, the chair is in charge again. Each packet also reports what the
 * body is doing (who decides, reference id, alert, motor/test, age of the
 * decision held), so the chair's CSV shows the applied state next to its own.
 *
 * Libraries: nrf_to_nrf (TMRh20, ESB; runs the RADIO directly, no SoftDevice),
 * PostureLink (including the checked Lsm6Accel reader over Wire1). As the transmitter,
 * radio.write() blocks until ACKed or out of retries (~20 ms) — fine here.
 *
 * Alert output: vibration motor on D10 (NPN low-side driver + flyback diode,
 * motor fed from 3V3; PWM duty MOTOR_DUTY sets the strength), mirrored on the
 * onboard red LED. Pulsed PULSE_ON_MS every PULSE_PERIOD_MS while alerting.
 * A motor test ('m' here, or 'm' on the chair) buzzes once (MOTOR_TEST_MS).
 *
 * Output (prose lines are '#'-prefixed; pandas read_csv(comment='#')), one
 * row per sample (20 Hz), written AFTER that sample's ACK was digested:
 *   t_ms,seq,ctl,cal,calid,dev,alert,motor,test,aseq,age,pitch,roll,imu,inh
 *   t_ms  = this board's millis() (never compare it with the chair's)
 *   seq   = BodyPacket.seq sent for this sample (boot id: header line 0)
 *   ctl   = who decides: 1 = chair (valid ChairAcks arriving), 2 = fallback
 *           (none for LINK_TIMEOUT_MS: body decides alone), 0 = none yet
 *   cal / calid = the body holds the chair's reference / its id (0 = none)
 *   dev   = degrees from that reference (0 if none)
 *   alert = alert in effect (chair's in ctl 1, local in ctl 2)
 *   motor = motor/LED output commanded on now (PWM; not a measurement)
 *   test  = a motor-test buzz is running (motor on for the test, not an alert)
 *   aseq / age = body sample the ChairAck held was decided on, and seq - aseq
 *           (1 normally: decided on the previous sample); age 254 = 254 or
 *           more; -1 / -1 = unknown (none held, or not made on a sample of
 *           this session). The next packet reports this row's ctl/cal/calid/
 *           alert/aseq to the chair (its age one higher), see PostureLink.h.
 *   imu = 0 valid, 1 init, 2 bus/short read, 3 no fresh data, 4 invalid vector
 *   inh = a chair alert was inhibited locally (invalid IMU or old decision)
 *   Invalid samples have nan dev/pitch/roll. Saved calibration is retained.
 * Events: "# link up/lost", "# alert ON/OFF", "# motor test (...)".
 * Stats, normally once per second (counts since the previous stats line; dt =
 * its span in ms, > 1000 only if the USB host stalled):
 *   # tx sent=N ack=A noack=F retx=R ctl=C bad=B ver=V sdrop=S edrop=E dt=T
 *   sent  = samples transmitted (radio.write calls)
 *   ack   = writes ACKed by the radio; noack = sent - ack (all retries spent —
 *           the chair may still have received it, only its ACK lost)
 *   retx  = retransmissions over all writes, failed ones included (+15 each)
 *   ctl   = ACKs carrying a valid ChairAck (these alone keep link = 1)
 *   bad   = ACKs with no/odd payload; ver = incompatible protocol (ack = ctl + bad + ver)
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
#include <RandomId.h>
#include <Lsm6Accel.h>
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
Lsm6Accel<decltype(Wire1)> imu(Wire1);         // checked I2C on the internal sensor bus

// chair link
uint32_t n = 0;                                // samples sent since boot; seq = low 16 bits
uint8_t bootId = 0;                            // BodyPacket.boot: random per boot
BodyLink chairLink;                            // .ack: CF_CAL/refs survive link loss (fallback)
unsigned long lastReport = 0;
bool radioFailed = false;                    // initialization failure in the header
bool imuReady = false, inhibited = false, versionNoted = false;
uint8_t imuStatus = IMU_INIT, failedSamples = 0;
uint32_t lastImuInit = 0;
ImuControlGuard imuGuard;

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
    case 1: if (!imuReady) l.str("# IMU not initialized"); return true;
    case 2: if (radioFailed) l.str("# radio init FAILED"); return true;
    case 3: l.str("t_ms,seq,ctl,cal,calid,dev,alert,motor,test,aseq,age,pitch,roll,imu,inh"); return true;
    case 4: l.str("# proto=").u32(PROTO_VER); return true;
  }
  return false;
}

void event(const char *s) { Line l; l.str(s); out.event(l); }

bool testRunning(unsigned long t) { return testUntil && (int32_t)(t - testUntil) < 0; }

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

// Send sample n and digest the ACK (if any) into chairLink. The packet also
// reports the state applied so far (at sample n-1): its ACK isn't in yet.
void sendSample(unsigned long t, float pitch, float roll) {
  const ChairAck &held = chairLink.ack;
  BodyPacket pkt = {};
  pkt.ver = PROTO_VER;
  pkt.boot = bootId;
  pkt.seq = (uint16_t)n;
  pkt.pitch = pitch;
  pkt.roll = roll;
  pkt.ctl = chairLink.control();
  pkt.imu = imuStatus;
  pkt.flags = (inhibited ? BF_INH : 0) | (alert ? BF_ALERT : 0) | (ledState ? BF_MOTOR : 0) | (testRunning(t) ? BF_TEST : 0);
  if (chairLink.haveAck) {
    pkt.calId = held.calId;
    if (held.flags & CF_CAL) pkt.flags |= BF_CAL;
  }
  pkt.age = chairLink.age(n);
  pkt.ackSeq = pkt.age == AGE_NONE ? 0 : held.basisSeq;

  bool ok = radio.write(&pkt, sizeof(pkt));
  uint8_t arc = radio.getARC();                // retransmissions, also when !ok
  uint8_t buf[32];                             // = the DPL max payload
  uint8_t len = 0;
  if (ok && radio.available()) {               // the payload of THIS ACK; read() clears it
    len = radio.getDynamicPayloadSize();
    if (len > sizeof(buf)) len = sizeof(buf);
    radio.read(buf, len);
  }

  BodyLink::Result r = chairLink.onTx(t, n, bootId, ok, arc, buf, len);
  if (r.version && !versionNoted) {
    versionNoted = true;
    Line l; l.str("# chair protocol v").u32(payloadVersion(buf, len))
             .str(" != v").u32(PROTO_VER).str(": flash both boards"); out.event(l);
  }
  if (r.buzz) motorTest("chair");
  if (r.linkChanged) {
    localGate.reset();
    event(chairLink.watch.up ? "# link up (chair decides)" : "# link lost: fallback, IMU only");
  }
}

// The installed Wire implementation has unbounded waits on a wedged bus.
// Switch the actuator off before entering it; a stuck call leaves PWM off.
// A returning transaction restores a manual test or the previous pulse only
// for a valid sample. No claim is made that software can recover a wedged bus.
uint8_t readImu(uint32_t t, float &pitch, float &roll) {
  bool wasOn = ledState;
  setLed(false);
  if (!imuReady && (uint32_t)(t - lastImuInit) >= 1000) {
    lastImuInit = t;
    imuReady = imu.begin() == IMU_OK;
    failedSamples = 0;
  }
  float ax = 0, ay = 0, az = 0;
  uint8_t status = imuReady ? imu.read(ax, ay, az) : (uint8_t)IMU_INIT;
  if (status == IMU_OK) {
    anglesFromAccel(ax, ay, az, pitch, roll);
    if (!anglesOk(pitch, roll)) status = IMU_RANGE;
  }
  if (status != IMU_OK) {
    pitch = roll = NAN;
    if (failedSamples < 20) failedSamples++;
    if (failedSamples >= 20) imuReady = false;
  } else failedSamples = 0;
  if (testRunning(t) || (status == IMU_OK && wasOn)) setLed(true);
  return status;
}

void printBodyRow(unsigned long t, float pitch, float roll, float dev, bool cal) {
  const ChairAck &ack = chairLink.ack;
  uint8_t age = chairLink.age(n);
  bool known = age != AGE_NONE;
  Line l;
  l.u32(t).chr(',').u32((uint16_t)n).chr(',').u32(chairLink.control()).chr(',')
   .u32(cal).chr(',').u32(chairLink.haveAck ? ack.calId : 0).chr(',').fix(dev, 1).chr(',')
   .u32(alert).chr(',').u32(ledState).chr(',').u32(testRunning(t)).chr(',')
   .i32(known ? (int32_t)ack.basisSeq : -1).chr(',').i32(known ? (int32_t)age : -1).chr(',')
   .fix(pitch, 1).chr(',').fix(roll, 1).chr(',').u32(imuStatus).chr(',').u32(inhibited);
  out.row(l);
}

void sample(unsigned long t) {
  float pitch = NAN, roll = NAN;
  uint8_t status = readImu(t, pitch, roll);
  if (status != imuStatus) {
    Line e; e.str("# IMU state=").u32(status); out.event(e);
  }
  imuStatus = status;
  bool valid = status == IMU_OK;
  imuGuard.observe(n, valid);

  sendSample(t, pitch, roll);
  bool linkUp = chairLink.watch.up;
  const ChairAck &ack = chairLink.ack;
  bool cal = ack.flags & CF_CAL;
  float dev = !valid ? NAN : cal ? deviation(pitch, roll, ack.pitchRef, ack.rollRef) : 0;

  inhibited = false;
  bool allowed = valid && imuGuard.allows(chairLink, n);
  if (!valid) {
    localGate.reset();
    inhibited = linkUp && cal && (ack.flags & CF_ALERT);
    setAlert(false, dev);
  } else if (linkUp) {
    inhibited = cal && (ack.flags & CF_ALERT) && !allowed;
    setAlert((ack.flags & CF_ALERT) && allowed, dev);
  } else if (cal) { localGate.update(dev, t); setAlert(localGate.alert, dev); }
  else setAlert(false, dev);
  if (testRunning(t)) setLed(true);           // manual test remains distinct from posture validity

  printBodyRow(t, pitch, roll, dev, cal);
  n++;
}

void report(unsigned long t) {
  Line l;
  l.str("# tx sent=").u32(chairLink.sent).str(" ack=").u32(chairLink.acked)
   .str(" noack=").u32(chairLink.sent - chairLink.acked).str(" retx=").u32(chairLink.retx)
   .str(" ctl=").u32(chairLink.ctl).str(" bad=").u32(chairLink.bad).str(" ver=").u32(chairLink.ver)
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

  bootId = randomId();
#if defined(PIN_LSM6DS3TR_C_POWER)
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
#if defined(NRF52840_XXAA)
  NRF_P1->PIN_CNF[8] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos)
                    | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                    | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                    | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
                    | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
#endif
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(10);                                  // sensor power-up, setup only
#endif
  Wire1.begin();
  imuReady = imu.begin() == IMU_OK;
  lastImuInit = millis();
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

  if (testRunning(t)) setLed(true);
  else {
    testUntil = 0;
    setLed(alert && (t - alertStart) % PULSE_PERIOD_MS < PULSE_ON_MS);
  }

  if (t - lastReport >= 1000) report(t);
}
