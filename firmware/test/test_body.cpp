// Integration test: the real firmware/body/body.ino on host stubs (stub/),
// with a fake radio deciding what each write() and its ACK payload look like.
// Starts 3 s before millis() wraps, so every scenario runs across the rollover.
#include "check.h"
#include "stub/Arduino.h"
#include "stub/nrf_to_nrf.h"
#include "stub/Wire.h"
FakeWire Wire1;

#include <vector>

namespace sim { uint32_t now = 0xFFFFFFFFu - 3000; int pin[32]; int adc = 0; float ax = 0, ay = 0, az = 1; }
FakeSerial Serial;
static FakeRng rng = {};
FakeRng *NRF_RNG = &rng;

#include "../body/body.ino"

static int wdtMisses = 0;                      // loop() iterations that didn't feed the watchdog

static void runFor(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    sim::now++; loop(); Serial.hostReads();
    wdtMisses += NRF_WDT->RR[0] != WDT_RR_RR_Reload;
    NRF_WDT->RR[0] = 0;
  }
}

static void tilt(float deg) {
  sim::ax = -sinf(deg / RAD_TO_DEGF); sim::ay = 0; sim::az = cosf(deg / RAD_TO_DEGF);
}

static void ackWith(uint8_t flags, uint8_t buzz = 0) {
  ChairAck a = {};
  a.ver = PROTO_VER; a.calId = 9;
  a.basisBoot = bootId; a.basisSeq = (uint16_t)n;
  a.flags = flags; a.buzz = buzz; a.pitchRef = 0; a.rollRef = 0;
  const uint8_t *b = (const uint8_t *)&a;
  radio.txOk = true; radio.txArc = 1;
  radio.txAck.assign(b, b + sizeof(a));
}
static void ackEmpty() { radio.txOk = true; radio.txArc = 0; radio.txAck.clear(); }
static void ackShort() { ackWith(CF_CAL); radio.txAck.resize(sizeof(ChairAck) - 1); }
static void txFail() { radio.txOk = false; radio.txArc = 15; radio.txAck.clear(); }

static std::vector<std::string> outLines() {
  std::vector<std::string> v;
  size_t p = 0, e;
  while ((e = Serial.out.find("\r\n", p)) != std::string::npos) { v.push_back(Serial.out.substr(p, e - p)); p = e + 2; }
  CHECK_EQ(p, Serial.out.size());
  return v;
}
static int count(const std::string &prefix) {
  int n = 0;
  for (auto &l : outLines()) n += l.compare(0, prefix.size(), prefix) == 0;
  return n;
}
static std::string lastRow() {
  auto v = outLines();
  for (size_t i = v.size(); i--;) if (!v[i].empty() && v[i][0] != '#') return v[i];
  return "";
}
// t_ms,seq,ctl,cal,calid,dev,alert,motor,test,aseq,age,pitch,roll
enum { F_SEQ = 1, F_CTL, F_CAL, F_CALID, F_DEV, F_ALERT, F_MOTOR, F_TEST, F_ASEQ, F_AGE };
static int rowField(int i) {                   // field i of the last row
  std::string r = lastRow();
  size_t p = 0;
  for (int k = 0; k < i; k++) p = r.find(',', p) + 1;
  return atoi(r.c_str() + p);
}
static long lastTx(const std::string &key) {
  long n = -1;
  for (auto &l : outLines()) {
    if (l.compare(0, 4, "# tx")) continue;
    size_t p = l.find(" " + key + "=");
    if (p != std::string::npos) n = atol(l.c_str() + p + key.size() + 2);
  }
  return n;
}

// Valid ChairAck once, then only ACKs without valid control: link must drop
// ~1 s after the last valid one, keeping the reference for the fallback.
static void dropsWithoutValidControl(void (*badAck)(), const char *what) {
  printf("  %s\n", what);
  ackWith(CF_CAL);
  int up0 = count("# link up"), lost0 = count("# link lost");
  runFor(100);
  CHECK_EQ(count("# link up"), up0 + 1);
  CHECK_EQ(rowField(F_CTL), CTL_CHAIR);
  badAck();
  runFor(900);
  CHECK_EQ(rowField(F_CTL), CTL_CHAIR);        // < 1 s since the last valid one
  runFor(200);
  CHECK_EQ(count("# link lost"), lost0 + 1);
  CHECK_EQ(rowField(F_CTL), CTL_FALLBACK);
  CHECK_EQ(rowField(F_CAL), 1);                // cal kept for the fallback
  CHECK_EQ(rowField(F_CALID), 9);
}

// Widest possible row (every field at its longest), at t_ms = UINT32_MAX:
// must go out whole through the real 256-byte USB FIFO.
static void testWidestRow() {
  ChairAck a = {};
  a.ver = PROTO_VER; a.flags = CF_CAL | CF_MUTE; a.calId = 255; a.pitchRef = 90; a.rollRef = 180;
  a.basisBoot = bootId; a.basisSeq = 65535 - 300;
  chairLink.ack = a;
  chairLink.haveAck = true;
  chairLink.basisOk = true;
  chairLink.basisN = 65535 - 300;              // age 300 -> 254 ("254 or more")
  chairLink.watch.up = false;                  // fallback (ctl 2)
  localGate.alert = true;                      // dev 402.5 keeps it on
  n = 65535;
  txFail();
  sim::ax = 1; sim::ay = -0.0f; sim::az = -1e-6f;   // pitch -90.0, roll -180.0
  Serial.hostReads();
  Serial.out.clear();
  sim::now = 0xFFFFFFFEu;
  lastSample = sim::now - SAMPLE_MS;
  lastReport = sim::now;
  testUntil = sim::now + 400;
  motorOn = true;
  sim::now++;
  imuStatus = IMU_OK; inhibited = false; alert = motorOn = true;
  battery.mv = 65535;
  printBodyRow(sim::now, -90, -180, 402.5, true);
  std::string row = lastRow();
  printf("  widest row (%zu bytes): %s\n", row.size(), row.c_str());
  CHECK_STR(row, "4294967295,65535,2,1,255,402.5,1,1,1,65235,254,-90.0,-180.0,0,0,65535,1");
  CHECK(row.size() + 2 <= (size_t)Line::CAP + 2);
  CHECK_EQ(out.rowDrops(), 0u);
  // stats line with every counter at its maximum (+ 9 digits each of slack
  // for ack/sdrop/edrop, 0 here); header lines all fit
  chairLink.sent = chairLink.acked = chairLink.retx = chairLink.ctl = chairLink.bad = chairLink.ver = 0xFFFFFFFFu;
  chairLink.acked = 0;                         // noack = sent: 10 digits too
  lastReport = sim::now + 1;
  report(sim::now);
  auto v = outLines();
  printf("  widest stats (%zu bytes + 27 for ack/sdrop/edrop): %s\n", v.back().size(), v.back().c_str());
  CHECK_STR(v.back().substr(0, 16), "# tx sent=429496");
  CHECK(v.back().size() + 27 <= Line::CAP);
  imuReady = false; radioFailed = true;
  for (uint8_t i = 0; i < 8; i++) { Line l; if (bodyHeader(i, l)) CHECK(l.ok()); }
}

static void testImuFailureAndRecovery() {
  tilt(20);
  ackWith(CF_CAL | CF_ALERT);
  runFor(100);
  CHECK(alert);
  uint8_t savedId = chairLink.ack.calId;
  Wire1.nackAll = true;
  runFor(50);
  CHECK_EQ(imuStatus, IMU_BUS);
  CHECK(!alert && !motorOn);
  CHECK_EQ(chairLink.control(), CTL_CHAIR);
  CHECK_EQ(chairLink.ack.calId, savedId);
  BodyPacket p;
  memcpy(&p, radio.sent.back().data(), sizeof p);
  CHECK_EQ(p.imu, IMU_BUS);
  CHECK(isnan(p.pitch) && isnan(p.roll));
  CHECK(parseBodyPacket(&p, sizeof p, p));
  Wire1.nackAll = false;
  runFor(200);
  CHECK_EQ(imuStatus, IMU_OK);
  CHECK(!alert && inhibited);                 // the ACK still names the pre-fault sample
  runFor(1000);
  CHECK(!alert && inhibited);
  ackWith(CF_CAL | CF_ALERT);
  runFor(50);
  CHECK(alert && !inhibited);                 // a new decision releases the guard

  txFail(); runFor(4300);
  CHECK_EQ(chairLink.control(), CTL_FALLBACK);
  CHECK(alert);
  Wire1.forceStale = true; runFor(50);
  CHECK(!alert && !motorOn);
  CHECK_EQ(imuStatus, IMU_STALE);
  Wire1.forceStale = false; runFor(2900);
  CHECK(!alert);                              // fallback hold starts over, not at pre-fault time
  runFor(250);
  CHECK(alert);

  Wire1.forceStale = true; runFor(50);
  Serial.in = "m"; runFor(60);
  CHECK(testRunning(sim::now) && motorOn && !alert);
  runFor(650);
  CHECK(!motorOn && !testRunning(sim::now));
  Wire1.forceStale = false;
  Wire1.reset(); runFor(1600);                // sensor brownout -> checked reconfiguration
  CHECK_EQ(imuStatus, IMU_OK);
  CHECK_EQ(chairLink.ack.calId, savedId);
  CHECK_EQ(Wire1.unbegun, 0);
}

static bool near(int mv, int want) { return abs(mv - want) <= 2; }   // rawForMv rounds

static int vbatReads() {
  BodyPacket p;
  memcpy(&p, radio.sent.back().data(), sizeof p);
  return p.vbat;
}

// Expect powerOff(): runs `ms` at most; true if the board entered System OFF.
static bool runUntilOff(uint32_t ms) {
  try { runFor(ms); } catch (SystemOff &) { return true; }
  return false;
}

static void checkPoweredOff(const char *why) {
  CHECK_EQ(sim::pin[MOTOR_PIN], 0);
  CHECK(!motorOn);
  CHECK(radio.poweredDown);
  for (int pin : {LED_RED, LED_GREEN, LED_BLUE}) CHECK_EQ(sim::pin[pin], HIGH);   // active LOW: off
  CHECK_EQ(sim::pin[VBAT_ENABLE], LOW);       // never HIGH, also in System OFF
  CHECK(!Wire1.begun);
  CHECK_EQ(pinModeOf(PIN_WIRE1_SDA), INPUT);   // no pull-ups left on the IMU's lines
  CHECK_EQ(pinModeOf(PIN_WIRE1_SCL), INPUT);
  CHECK_EQ(count(std::string("# battery low (") + why + ")"), 1);
}

// One noisy low reading, readings only with the motor rested, and USB.
static void testBatteryNoiseAndMotorSag() {
  sim::vbat() = sim::rawForMv(3700);
  runFor(2100);
  CHECK(near(vbatReads(), 3700));
  sim::vbat() = sim::rawForMv(3100);         // one low reading...
  runFor(2000);
  CHECK(battery.lows >= 1);
  sim::vbat() = sim::rawForMv(3700);         // ...then fine: no cutoff, count restarts
  runFor(2100);
  CHECK_EQ(battery.lows, 0);
  // continuous alert: the motor pulses 700 ms / 1 s. Its ~90 mA sag shows up
  // while it runs and for 150 ms after; no reading may see it
  ChairAck a = {};
  a.ver = PROTO_VER; a.calId = 9; a.flags = CF_CAL | CF_ALERT;
  tilt(20);
  uint32_t reads = 0, lastAt = battery.lastAt;
  uint32_t offAt = sim::now;
  radio.writeMs = 3;                         // write() runs past loop()'s t (retries)
  for (int i = 0; i < 20000; i++) {
    a.basisBoot = bootId; a.basisSeq = (uint16_t)n;
    const uint8_t *b = (const uint8_t *)&a;
    radio.txOk = true; radio.txArc = 0; radio.txAck.assign(b, b + sizeof a);
    if (motorOn) offAt = sim::now;
    sim::vbat() = sim::rawForMv(motorOn || sim::now - offAt < 150 ? 3000 : 3700);
    runFor(1);
    if (battery.lastAt != lastAt) {
      reads++;
      CHECK(near(battery.mv, 3700));
      battery.lastAt -= 137;                   // the 2 s period is in phase with the 1 s pulse:
      lastAt = battery.lastAt;                 // shift it so readings fall due at every phase
    }
  }
  radio.writeMs = 0;
  CHECK(alert);
  CHECK(reads >= 9);                         // ~every 2 s, between pulses
  CHECK_EQ(battery.lows, 0);
  // a buzz ending on a sample iteration whose radio.write() runs 5 ms past
  // loop()'s t: the motor goes off after t, and no reading may come before
  // 200 ms after that real off time
  tilt(0); ackEmpty(); runFor(1500);
  Serial.in = "m"; runFor(100);
  CHECK(motorOn);
  battery.lastAt = sim::now - 10 * VBAT_PERIOD_MS;   // a reading is overdue
  testUntil = sim::now + 1;                    // the buzz ends on the next loop()...
  lastSample = sim::now + 1 - SAMPLE_MS;       // ...which also samples
  radio.writeMs = 5;
  sim::vbat() = sim::rawForMv(3000);           // sagging until 200 ms after off
  uint32_t before = battery.lastAt;
  runFor(1);
  radio.writeMs = 0;
  CHECK(!motorOn);
  CHECK_EQ(battery.lastAt, before);            // no reading right at the off edge
  uint32_t off = motorOffAt;
  CHECK(off != 0);
  while (sim::now - off < VBAT_MOTOR_REST_MS - 1) { runFor(1); CHECK_EQ(battery.lastAt, before); }
  sim::vbat() = sim::rawForMv(3700);
  runFor(5);
  CHECK(battery.lastAt != before && near(battery.mv, 3700));
  // on USB the cell can read low (no load on it, charger): never a cutoff
  tilt(0); ackEmpty();
  fakePower().USBREGSTATUS = POWER_USBREGSTATUS_VBUSDETECT_Msk;
  sim::vbat() = sim::rawForMv(3000);
  runFor(10000);
  CHECK_EQ(battery.lows, 0);
  CHECK(near(vbatReads(), 3000));
  fakePower().USBREGSTATUS = 0;
  sim::vbat() = sim::rawForMv(3900);
  runFor(2100);
}

// On battery below the cutoff: 3 readings in a row (~4-6 s) -> System OFF.
static void testBatteryCutoff() {
  Serial.hostReads();
  ChairAck a = {};
  a.ver = PROTO_VER; a.calId = 9; a.flags = CF_CAL | CF_ALERT;
  radio.txOk = true; radio.txAck.assign((const uint8_t *)&a, (const uint8_t *)&a + sizeof a);
  sim::vbat() = sim::rawForMv(3280);
  uint32_t t0 = sim::now;
  bool off = runUntilOff(10000);
  CHECK(off);
  uint32_t took = sim::now - t0;
  CHECK(took >= 2 * VBAT_PERIOD_MS && took <= 3 * VBAT_PERIOD_MS + 1500);
  CHECK(near(battery.mv, 3280));
  checkPoweredOff("cutoff");
}

static int testBootLow() {
  for (int &pin : sim::pin) pin = HIGH;
  sim::vbat() = sim::rawForMv(3350);          // above the cutoff, below the boot level
  bool off = false;
  try { setup(); } catch (SystemOff &) { off = true; }
  CHECK(off);
  CHECK_EQ(NRF_WDT->RUNSTATUS, 0u);           // went off before anything else started
  CHECK(radio.sent.empty());
  CHECK_EQ(sim::pin[MOTOR_PIN], 0);
  CHECK(radio.poweredDown);
  for (int pin : {LED_RED, LED_GREEN, LED_BLUE}) CHECK_EQ(sim::pin[pin], HIGH);
  CHECK_EQ(sim::pin[VBAT_ENABLE], LOW);
  return report("body.ino boot on a low battery");
}

// Low battery but USB present: boots and runs; unplugged, it cuts off.
static int testBootUsb() {
  for (int &pin : sim::pin) pin = HIGH;
  fakePower().USBREGSTATUS = POWER_USBREGSTATUS_VBUSDETECT_Msk;
  sim::vbat() = sim::rawForMv(3200);
  bool off = false;
  try { setup(); runFor(10000); } catch (SystemOff &) { off = true; }
  CHECK(!off);
  CHECK(radio.sent.size() >= 190);
  CHECK(near(vbatReads(), 3200));
  CHECK_EQ(wdtMisses, 0);
  fakePower().USBREGSTATUS = 0;
  CHECK(runUntilOff(10000));
  checkPoweredOff("cutoff");
  return report("body.ino boot on USB with a low battery");
}

static void assertMotorOffDuringI2c() { CHECK_EQ(sim::pin[MOTOR_PIN], 0); }

static int testInitFailure() {
  rng.VALUE = 0x5a;
  for (int &pin : sim::pin) pin = HIGH;
  Wire1.nackAll = true;
  Wire1.onTransaction = assertMotorOffDuringI2c;
  setup();
  runFor(3200);                              // boot failed, radio heartbeat still crosses millis wrap
  CHECK(!imuReady);
  CHECK_EQ(imuStatus, IMU_INIT);
  CHECK(n >= 63 && n <= 65);
  CHECK(Wire1.tx <= 5);                       // reinitialization is at most once per second
  CHECK(!alert && !motorOn);
  CHECK_EQ(rowField(F_CAL), 0);
  BodyPacket p;
  memcpy(&p, radio.sent.back().data(), sizeof p);
  CHECK_EQ(p.imu, IMU_INIT);
  CHECK(isnan(p.pitch) && isnan(p.roll));
  CHECK(parseBodyPacket(&p, sizeof p, p));
  Wire1.nackAll = false;
  Wire1.regs[0x0F] = 0x00;
  runFor(1100);
  CHECK(!imuReady);
  Wire1.regs[0x0F] = 0x6a;
  runFor(1200);
  CHECK(imuReady);
  CHECK_EQ(imuStatus, IMU_OK);
  CHECK(!alert && !motorOn);
  CHECK_EQ(Wire1.unbegun, 0);
  return report("body.ino boot failure");
}

// Highest motor PWM duty seen while running ms more.
static int peakPwm(uint32_t ms) {
  int peak = 0;
  for (uint32_t i = 0; i < ms; i++) { runFor(1); peak = std::max(peak, sim::pin[MOTOR_PIN]); }
  return peak;
}

// Chair mute (CF_MUTE): alerts keep the PWM at 0 but are still followed and
// reported (motor = would buzz); a motor test still buzzes; the mute is held
// in fallback; unmuting brings the buzz back.
static void testMotorMute() {
  int on0 = count("# motor muted (chair)"), off0 = count("# motor unmuted (chair)");
  tilt(0);
  ackWith(CF_CAL | CF_ALERT | CF_MUTE);
  CHECK_EQ(peakPwm(1100), 0);
  CHECK_EQ(count("# motor muted (chair)"), on0 + 1);
  CHECK_EQ(rowField(F_ALERT), 1);
  CHECK_EQ(rowField(16), 1);                   // mute column
  bool wouldBuzz = false;
  for (size_t i = radio.sent.size() - 20; i < radio.sent.size(); i++) {
    BodyPacket q;
    memcpy(&q, radio.sent[i].data(), sizeof(q));
    CHECK(q.flags & BF_MUTE);
    wouldBuzz |= (q.flags & BF_MOTOR) != 0;
  }
  CHECK(wouldBuzz);                            // BF_MOTOR / motor column: when it would buzz
  ackWith(CF_CAL | CF_MUTE);                   // no alert: a motor test still buzzes
  runFor(200);
  Serial.in = "m";
  CHECK_EQ(peakPwm(100), MOTOR_DUTY);
  CHECK_EQ(peakPwm(600), MOTOR_DUTY);          // until the test ends...
  CHECK_EQ(peakPwm(100), 0);                   // ...then off again
  // link lost: the fallback alerts with the reference held, still muted
  txFail();
  tilt(20);
  CHECK_EQ(peakPwm(4500), 0);
  CHECK_EQ(rowField(F_CTL), CTL_FALLBACK);
  CHECK_EQ(rowField(F_ALERT), 1);
  CHECK_EQ(rowField(16), 1);
  // the chair unmutes: the alert buzzes again
  tilt(0);
  ackWith(CF_CAL | CF_ALERT);
  CHECK_EQ(peakPwm(1100), MOTOR_DUTY);
  CHECK_EQ(count("# motor unmuted (chair)"), off0 + 1);
  CHECK_EQ(rowField(16), 0);
  ackWith(CF_CAL);
  runFor(200);
  CHECK_EQ(sim::pin[MOTOR_PIN], 0);
}

// The buzz shakes the IMU: while the PWM runs the body sends (and logs) the
// last reading taken with it off; with the motor off, live readings again.
static void testShakeHold() {
  tilt(15);
  ackWith(CF_CAL);
  runFor(200);
  ackWith(CF_CAL | CF_ALERT);                  // the chair alerts: the motor pulses
  for (int i = 0; i < 2000 && !motorDriven; i++) runFor(1);
  CHECK(motorDriven);
  tilt(0);                                     // what a shaken reading may show
  int held = 0, live = 0;
  float last = 15;                             // read before the motor started
  for (int i = 0; i < 1500; i++) {
    bool shaking = motorDriven;                // as readImu() sees it in this loop()
    size_t k = radio.sent.size();
    runFor(1);
    if (radio.sent.size() == k) continue;
    BodyPacket q;
    memcpy(&q, radio.sent.back().data(), sizeof q);
    if (shaking) { CHECK(fabsf(q.pitch - last) < 0.1f); held++; }   // the last quiet one
    else { CHECK(fabsf(q.pitch) < 0.1f); live++; last = q.pitch; }
  }
  CHECK(held >= 10);
  CHECK(live >= 3);
  CHECK_EQ(alert, 1);                          // the alert itself is untouched
  ackWith(CF_CAL);
  runFor(200);
  CHECK_EQ(sim::pin[MOTOR_PIN], 0);
}

int main(int argc, char **argv) {
  resetReasonSim() = 4;                       // soft reset (e.g. after an upload)
  if (argc > 1 && !strcmp(argv[1], "bootlow")) return testBootLow();
  if (argc > 1 && !strcmp(argv[1], "bootusb")) return testBootUsb();
  if (argc > 1) return testInitFailure();
  Wire1.onTransaction = assertMotorOffDuringI2c;
  rng.VALUE = 0x5a;
  for (int &p : sim::pin) p = HIGH;
  setup();
  runFor(100);
  uint32_t idleWrites = analogWriteCalls();
  CHECK(idleWrites > 0);                      // setup must explicitly command motor off
  runFor(250);
  CHECK_EQ(analogWriteCalls(), idleWrites);   // idle loop and I2C guard never restart PWM at duty zero
  auto v = outLines();
  CHECK(v.size() >= 2);
  CHECK(v[0].find("boot=5a") != std::string::npos);
  CHECK_STR(v[1], "t_ms,seq,ctl,cal,calid,dev,alert,motor,test,aseq,age,pitch,roll,imu,inh,vbat,mute");
  CHECK_STR(v[2], "# proto=5");
  CHECK_STR(v[3], "# resetreas=0x000004");
  CHECK_STR(v[4], "# vbat=3899 mV usb=0");
  // watchdog started once, 4 s, fed from setup on
  CHECK_EQ(NRF_WDT->RUNSTATUS, 1u);
  CHECK_EQ(NRF_WDT->CRV, 4u * 32768 - 1);
  CHECK_EQ(NRF_WDT->RREN, (uint32_t)WDT_RREN_RR0_Msk);
  CHECK_EQ(sim::pin[VBAT_ENABLE], LOW);
  CHECK_EQ(sim::pin[PIN_CHARGING_CURRENT], LOW);
  CHECK(!radio.sent.empty());
  BodyPacket p0, chk;
  CHECK_EQ(radio.sent.back().size(), sizeof(BodyPacket));
  memcpy(&p0, radio.sent.back().data(), sizeof(p0));
  CHECK_EQ(p0.ver, PROTO_VER);
  CHECK_EQ(p0.boot, 0x5a);
  CHECK(parseBodyPacket(&p0, sizeof p0, chk));
  CHECK_EQ(p0.ctl, CTL_NONE);
  CHECK_EQ(p0.age, AGE_NONE);
  CHECK_EQ(p0.vbat, 3899);

  // no valid ChairAck ever: radio ACKs alone never bring the link up
  ackEmpty();
  runFor(500);
  CHECK_EQ(count("# link up"), 0);
  CHECK_EQ(rowField(F_CTL), CTL_NONE);
  CHECK_EQ(rowField(F_AGE), -1);
  CHECK_EQ(rowField(F_ASEQ), -1);
  // a v1 chair's 12-byte ChairAck: not control either
  ackWith(CF_CAL);
  radio.txAck.resize(12);
  runFor(500);
  CHECK_EQ(count("# link up"), 0);

  dropsWithoutValidControl(ackEmpty, "empty ACKs");
  dropsWithoutValidControl(ackShort, "wrong-size ACKs");
  dropsWithoutValidControl([] { ackWith(0x80 | CF_CAL); }, "ACKs with unknown flags");

  // fallback decides alone with the kept reference: slouch > 3 s -> local alert
  ackEmpty();
  tilt(20);
  int on0 = count("# alert ON");
  runFor(3500);
  CHECK_EQ(count("# alert ON"), on0 + 1);
  CHECK_EQ(rowField(F_CTL), CTL_FALLBACK);
  CHECK_EQ(rowField(F_ALERT), 1);
  tilt(0);
  runFor(PULSE_PERIOD_MS);                     // the first reading with the motor off clears it
  CHECK_EQ(rowField(F_ALERT), 0);

  // radio failures: every retry counted (15 per failed write), noack = sent
  txFail();
  runFor(2100);
  CHECK(lastTx("sent") >= 19);
  CHECK_EQ(lastTx("ack"), 0);
  CHECK_EQ(lastTx("noack"), lastTx("sent"));
  CHECK_EQ(lastTx("retx"), 15 * lastTx("sent"));

  // recovery: link up, a buzz requested while away is only synced...
  int test0 = count("# motor test (chair)");
  ackWith(CF_CAL, 5);
  runFor(1100);
  CHECK_EQ(rowField(F_CTL), CTL_CHAIR);
  CHECK_EQ(count("# motor test (chair)"), test0);
  CHECK_EQ(lastTx("ctl"), lastTx("ack"));
  CHECK_EQ(lastTx("bad"), 0);
  // ...a new request while linked buzzes once; the chair's alert is followed
  ackWith(CF_CAL | CF_ALERT, 6);
  runFor(200);
  CHECK_EQ(count("# motor test (chair)"), test0 + 1);
  CHECK_EQ(rowField(F_ALERT), 1);
  // mixed valid / empty ACKs are told apart: every stats line adds up
  for (int i = 0; i < 40; i++) { if (i % 2) ackEmpty(); else ackWith(CF_CAL, 6); runFor(50); }
  int mixed = 0;
  for (auto &l : outLines()) {
    if (l.compare(0, 4, "# tx")) continue;
    long sent, ack, noack, retx, ctl, bad, ver;
    CHECK_EQ(sscanf(l.c_str(), "# tx sent=%ld ack=%ld noack=%ld retx=%ld ctl=%ld bad=%ld ver=%ld",
                    &sent, &ack, &noack, &retx, &ctl, &bad, &ver), 7);
    CHECK_EQ(ack, ctl + bad + ver);
    CHECK_EQ(noack, sent - ack);
    mixed += ctl > 0 && bad > 0;
  }
  CHECK(mixed >= 1);
  CHECK_EQ(rowField(F_CTL), CTL_CHAIR);        // a valid one every 100 ms keeps it up
  // manual motor test ('m' here), no alert: motor + test, alert stays 0 —
  // in the row now and in the next packet
  tilt(0);
  runFor(500);
  CHECK_EQ(rowField(F_ALERT), 0);
  Serial.in = "m";
  runFor(60);
  CHECK_EQ(rowField(F_MOTOR), 1);
  CHECK_EQ(rowField(F_TEST), 1);
  CHECK_EQ(rowField(F_ALERT), 0);
  BodyPacket pt;
  memcpy(&pt, radio.sent.back().data(), sizeof(pt));
  CHECK_EQ(pt.flags & (BF_MOTOR | BF_TEST | BF_ALERT), BF_MOTOR | BF_TEST);
  runFor(600);
  CHECK_EQ(rowField(F_MOTOR), 0);
  CHECK_EQ(rowField(F_TEST), 0);
  testMotorMute();
  testShakeHold();
  // every packet sent in this run is a well-formed packet
  int badSent = 0;
  for (auto &b : radio.sent) badSent += !parseBodyPacket(b.data(), (uint8_t)b.size(), chk);
  CHECK_EQ(badSent, 0);
  CHECK(sim::now < 0xFFFFFFFFu - 3000);        // the run crossed the millis() wrap
  CHECK_EQ(Serial.blocked, 0);
  outLines();
  testImuFailureAndRecovery();
  try { testBatteryNoiseAndMotorSag(); } catch (SystemOff &) { CHECK(!"false battery cutoff"); return report("body.ino"); }
  CHECK_EQ(wdtMisses, 0);
  testWidestRow();
  CHECK_EQ(Serial.blocked, 0);
  testBatteryCutoff();
  return report("body.ino");
}
