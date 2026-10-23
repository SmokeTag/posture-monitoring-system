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
 * motor fed from 3V3; PWM duty MOTOR_DUTY sets the strength). Pulsed
 * PULSE_ON_MS every PULSE_PERIOD_MS while alerting. A motor test ('m' here,
 * or 'm' on the chair) buzzes once (MOTOR_TEST_MS).
 * Motor shake: the buzz shakes the IMU (±10° in its first ~100-150 ms; it cleared
 * alerts in the 2026-10-22 validation), so a reading taken while the PWM runs
 * is replaced by the last one taken with it off (for at most SHAKE_HOLD_MAX_MS):
 * pitch/roll (row and packet) hold still while the motor runs.
 * Motor mute (data captures, 'q' on the chair): while the held ChairAck has
 * CF_MUTE, alert pulses keep the PWM at 0; the alert logic, LED and `motor`
 * column run as usual (= would buzz) and a motor test still buzzes. Held
 * through link loss like the reference; never saved: a reboot unmutes.
 *
 * Status LED (onboard RGB, dimmed to LED_LEVEL for the battery; colours in
 * PostureLink/StatusLed.h): red while the motor runs; otherwise red fast =
 * radio/IMU fault, orange = no link, blue = linked, green = linked and
 * calibrated (off between alert pulses while linked); 3 magenta blinks =
 * battery low, powering off.
 *
 * Battery protection (rule and calibration: PostureLink/Battery.h): VBAT is
 * read every 2 s while the motor rests and reported to the chair (vbat). On
 * battery, 3 readings in a row below 3.30 V stop motor + radio, blink, and
 * enter System OFF (~µA, RAM lost). A boot on battery below 3.40 V goes
 * straight back to System OFF. Never cuts off while USB VBUS is present.
 * Wake: the reset button, or plugging USB in (VBUS detect is an nRF52840
 * System OFF wake source, always on; an unplug may wake it too, and the boot
 * check sends it back). A watchdog (WDT_TIMEOUT_S, fed every loop()) resets
 * the board if anything hangs, so a hang can't leave the cutoff dead.
 *
 * Output (prose lines are '#'-prefixed; pandas read_csv(comment='#')), one
 * row per sample (20 Hz), written AFTER that sample's ACK was digested:
 *   t_ms,seq,ctl,cal,calid,dev,alert,motor,test,aseq,age,pitch,roll,imu,inh,vbat,mute
 *   t_ms  = this board's millis() (never compare it with the chair's)
 *   seq   = BodyPacket.seq sent for this sample (boot id: header line 0)
 *   ctl   = who decides: 1 = chair (valid ChairAcks arriving), 2 = fallback
 *           (none for LINK_TIMEOUT_MS: body decides alone), 0 = none yet
 *   cal / calid = the body holds the chair's reference / its id (0 = none)
 *   dev   = degrees from that reference (0 if none)
 *   alert = alert in effect (chair's in ctl 1, local in ctl 2)
 *   motor = motor output commanded on now (not a measurement); with mute,
 *           when it would buzz (the PWM stays 0 unless a test runs)
 *   test  = a motor-test buzz is running (motor on for the test, not an alert)
 *   aseq / age = body sample the ChairAck held was decided on, and seq - aseq
 *           (1 normally: decided on the previous sample); age 254 = 254 or
 *           more; -1 / -1 = unknown (none held, or not made on a sample of
 *           this session). The next packet reports this row's ctl/cal/calid/
 *           alert/aseq to the chair (its age one higher), see PostureLink.h.
 *   imu = 0 valid, 1 init, 2 bus/short read, 3 no fresh data, 4 invalid vector
 *   inh = a chair alert was inhibited locally (invalid IMU or old decision)
 *   vbat = battery mV, last reading (0 = none yet; on USB: the charger output)
 *   mute = motor muted by the chair (CF_MUTE held): alerts don't buzz
 *   Invalid samples have nan dev/pitch/roll. Saved calibration is retained.
 * Events: "# link up/lost", "# alert ON/OFF", "# motor test (...)", "# motor muted/unmuted",
 * "# battery low (...)" just before System OFF (only seen if a host is there).
 * Header: "# resetreas=0x..." (POWER->RESETREAS at boot: 1 pin, 2 watchdog,
 * 4 soft reset, 0x100000 VBUS wake) and "# vbat=... usb=...".
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
 *   BODY=$(readlink -f /dev/serial/by-id/usb-Seeed_XIAO_nRF52840_Sense_3B4A158C694F1777-if00)
 *   arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p "$BODY" firmware/body
 *   arduino-cli monitor -p "$BODY" -c baudrate=115200
 */

#include <Adafruit_TinyUSB.h>                  // needed for USB Serial on this core
#include <nrf_to_nrf.h>
#include <PostureLink.h>
#include <SerialOut.h>
#include <RandomId.h>
#include <Lsm6Accel.h>
#include <StatusLed.h>
#include <Battery.h>
#include "Wire.h"
#include <math.h>

using namespace posture;

const unsigned long SAMPLE_MS = 50;            // 20 Hz

// Alert output: motor driver (active HIGH, PWM)
const int MOTOR_PIN  = D10;
const int MOTOR_DUTY = 255;                    // 0..255 (255 = always on); lower = weaker buzz
const uint8_t LED_LEVEL = 32;                  // status LED brightness 0..255 (LiPo: ~1/8)
const unsigned long IMU_FAULT_MS = 250;        // no valid IMU sample this long = fault
const unsigned long PULSE_PERIOD_MS = 1000, PULSE_ON_MS = 700;
const unsigned long MOTOR_TEST_MS   = 500;
const unsigned long SHAKE_HOLD_MAX_MS = 1500;  // > a test run into a pulse (<= ~1 s): then the shaken reading
const unsigned long LOWBAT_BLINK_MS = 150;     // battery-low blink: 3 x (on, off)
const uint32_t WDT_TIMEOUT_S = 4;              // >> a radio.write() with all retries (~20 ms)

const float RAD_TO_DEGF = 57.2957795f;

nrf_to_nrf radio;
StatusLed led(LED_RED, LED_GREEN, LED_BLUE, LED_LEVEL);
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
uint32_t imuOkAt = 0;                          // last valid IMU sample (status LED)
float quietPitch = NAN, quietRoll = NAN;       // last valid reading with the motor off...
uint32_t quietAt = 0;                          // ...and when
ImuControlGuard imuGuard;

bool bodyHeader(uint8_t i, Line &l);
bool usbPresent();
SerialOut<decltype(Serial)> out(Serial, bodyHeader);

// alert state
AlertGate localGate;                           // only used in fallback
bool alert = false, motorOn = false;          // motorOn: commanded (with mute: would buzz)
bool motorDriven = false;                      // PWM actually on
bool outputInitialized = false;
bool mutedShown = false;                       // mute state last reported as an event
unsigned long alertStart = 0;
unsigned long lastSample = 0;
unsigned long testUntil = 0;                   // motor test buzz; 0 = idle

// battery
BatteryMonitor battery;
uint32_t motorOffAt = 0;                       // millis() the motor last went off

// Axis mapping for this mounting (see archive/xiao_imu_test): pitch = forward/back.
void anglesFromAccel(float aX, float aY, float aZ, float &pitch, float &roll) {
  pitch = atan2f(-aX, sqrtf(aY * aY + aZ * aZ)) * RAD_TO_DEGF;
  roll  = atan2f(aY, aZ) * RAD_TO_DEGF;
}

bool testRunning(unsigned long t) { return testUntil && (int32_t)(t - testUntil) < 0; }

// Muted by the chair (CF_MUTE in the ChairAck held): a reboot clears it.
bool motorMuted() { return chairLink.haveAck && (chairLink.ack.flags & CF_MUTE); }

void setMotor(bool on) {
  bool drive = on && (!motorMuted() || testRunning(millis()));
  // analogWrite restarts a hardware PWM sequence even for the same duty.
  // Do not flood the peripheral from the idle loop; always initialize it once.
  if (outputInitialized && drive == motorDriven) { motorOn = on; return; }
  outputInitialized = true;
  if (motorDriven && !drive) motorOffAt = millis();
  motorOn = on;
  motorDriven = drive;
  analogWrite(MOTOR_PIN, drive ? MOTOR_DUTY : 0);
}

void updateStatusLed(unsigned long t) {
  bool linkUp = chairLink.watch.up;
  if (motorOn) led.set(RGB_RED);
  else if (radioFailed || (uint32_t)(t - imuOkAt) >= IMU_FAULT_MS)
    led.set(phaseOn(t, 0, 2 * BLINK_FAST_MS, BLINK_FAST_MS) ? RGB_RED : RGB_OFF);
  else if (!linkUp) led.set(RGB_ORANGE);
  else if (alert) led.set(RGB_OFF);            // between pulses
  else led.set(chairLink.ack.flags & CF_CAL ? RGB_GREEN : RGB_BLUE);
}

bool bodyHeader(uint8_t i, Line &l) {
  switch (i) {
    case 0: l.str("# body unit: IMU -> chair (the brain). Calibrate on the chair; 'm' buzzes. boot=")
             .hex8(bootId); return true;
    case 1: if (!imuReady) l.str("# IMU not initialized"); return true;
    case 2: if (radioFailed) l.str("# radio init FAILED"); return true;
    case 3: l.str("t_ms,seq,ctl,cal,calid,dev,alert,motor,test,aseq,age,pitch,roll,imu,inh,vbat,mute"); return true;
    case 4: l.str("# proto=").u32(PROTO_VER); return true;
    case 5: {
      uint32_t r = readResetReason();
      l.str("# resetreas=0x").hex8(r >> 16).hex8(r >> 8).hex8(r);
      return true;
    }
    case 6: l.str("# vbat=").u32(battery.mv).str(" mV usb=").u32(usbPresent()); return true;
  }
  return false;
}

void event(const char *s) { Line l; l.str(s); out.event(l); }

void setAlert(bool on, float dev) {
  if (on == alert) return;
  alert = on;
  alertStart = millis();
  Line l;
  l.str(on ? "# alert ON  dev=" : "# alert OFF dev=").fix(dev, 1);
  out.event(l);
  if (!on) setMotor(false);
}

void motorTest(const char *src) {
  testUntil = millis() + MOTOR_TEST_MS;
  setMotor(true);
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
  pkt.flags = (inhibited ? BF_INH : 0) | (alert ? BF_ALERT : 0) | (motorOn ? BF_MOTOR : 0) | (testRunning(t) ? BF_TEST : 0)
            | (motorMuted() ? BF_MUTE : 0);
  if (chairLink.haveAck) {
    pkt.calId = held.calId;
    if (held.flags & CF_CAL) pkt.flags |= BF_CAL;
  }
  pkt.age = chairLink.age(n);
  pkt.ackSeq = pkt.age == AGE_NONE ? 0 : held.basisSeq;
  pkt.vbat = battery.mv;

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
  if (motorMuted() != mutedShown) {
    mutedShown = !mutedShown;
    event(mutedShown ? "# motor muted (chair): alerts don't buzz" : "# motor unmuted (chair)");
  }
}

// The installed Wire implementation has unbounded waits on a wedged bus.
// Switch the actuator off before entering it; a stuck call leaves PWM off.
// A returning transaction restores a manual test or the previous pulse only
// for a valid sample. No claim is made that software can recover a wedged bus.
uint8_t readImu(uint32_t t, float &pitch, float &roll) {
  bool wasOn = motorOn, shaking = motorDriven;
  setMotor(false);
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
  if (status != IMU_OK) quietPitch = quietRoll = NAN;
  else if (!shaking) { quietPitch = pitch; quietRoll = roll; quietAt = t; }
  else if (floatIsFinite(quietPitch) && (uint32_t)(t - quietAt) <= SHAKE_HOLD_MAX_MS) {
    pitch = quietPitch; roll = quietRoll;      // the motor was running: shaken reading
  }
  if (testRunning(t) || (status == IMU_OK && wasOn)) setMotor(true);
  return status;
}

void printBodyRow(unsigned long t, float pitch, float roll, float dev, bool cal) {
  const ChairAck &ack = chairLink.ack;
  uint8_t age = chairLink.age(n);
  bool known = age != AGE_NONE;
  Line l;
  l.u32(t).chr(',').u32((uint16_t)n).chr(',').u32(chairLink.control()).chr(',')
   .u32(cal).chr(',').u32(chairLink.haveAck ? ack.calId : 0).chr(',').fix(dev, 1).chr(',')
   .u32(alert).chr(',').u32(motorOn).chr(',').u32(testRunning(t)).chr(',')
   .i32(known ? (int32_t)ack.basisSeq : -1).chr(',').i32(known ? (int32_t)age : -1).chr(',')
   .fix(pitch, 1).chr(',').fix(roll, 1).chr(',').u32(imuStatus).chr(',').u32(inhibited)
   .chr(',').u32(battery.mv).chr(',').u32(motorMuted());
  out.row(l);
}

void sample(unsigned long t) {
  float pitch = NAN, roll = NAN;
  uint8_t status = readImu(t, pitch, roll);
  if (status != imuStatus) {
    Line e; e.str("# IMU state=").u32(status); out.event(e);
  }
  imuStatus = status;
  if (status == IMU_OK) imuOkAt = t;
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
  if (testRunning(t)) setMotor(true);           // manual test remains distinct from posture validity

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

// TinyUSB stops the 16 MHz crystal when USB is unplugged (dcd_nrf5x.c,
// USB_EVT_REMOVED -> hfclk_disable); nrf_to_nrf starts it only in begin().
// On the internal RC clock the radio is off frequency: unplugging USB while
// on the LiPo would drop the link until a reset. Restart it whenever it stops.
void keepCrystal() {
#if defined(NRF52840_XXAA)
  const uint32_t xtalOn = CLOCK_HFCLKSTAT_SRC_Msk | CLOCK_HFCLKSTAT_STATE_Msk;
  if ((NRF_CLOCK->HFCLKSTAT & xtalOn) != xtalOn) NRF_CLOCK->TASKS_HFCLKSTART = 1;
#endif
}

bool usbPresent() { return NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk; }

// Watchdog: a hang anywhere (e.g. Wire's unbounded waits on a wedged bus)
// resets the board instead of leaving the battery cutoff dead; the next boot
// runs the battery check before anything that could hang again. Fed every
// loop() and in setup()'s USB wait. Runs while the CPU sleeps, pauses under a
// debugger. Once started it can't be stopped and survives soft resets (the
// bootloader feeds it during DFU); a running one is only fed.
void feedWatchdog() { NRF_WDT->RR[0] = WDT_RR_RR_Reload; }

void startWatchdog() {
  if (!(NRF_WDT->RUNSTATUS & WDT_RUNSTATUS_RUNSTATUS_Msk)) {
    NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos)
                    | (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
    NRF_WDT->CRV = WDT_TIMEOUT_S * 32768 - 1;  // 32.768 kHz ticks
    NRF_WDT->RREN = WDT_RREN_RR0_Msk;
    NRF_WDT->TASKS_START = 1;
  }
  feedWatchdog();
}

#if defined(PIN_VBAT)
uint16_t readVbatMv() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < VBAT_SAMPLES; i++) sum += analogRead(PIN_VBAT);
  return vbatMv(sum, VBAT_SAMPLES);
}
#endif

// End of the battery protection: motor + radio off, a brief dim blink, then
// System OFF. GPIO latches hold there: motor LOW, LED HIGH (off), IMU power
// LOW, P0.14 stays LOW (the divider's ~2.8 µA; HIGH could harm P0.31). The
// SoftDevice is never enabled (nrf_to_nrf owns the RADIO), so SYSTEMOFF is
// written directly. Waking is a reset: setup() checks the battery again.
void powerOff(const char *why) {
  setMotor(false);
  digitalWrite(MOTOR_PIN, LOW);
  Line l;
  l.str("# battery low (").str(why).str("): vbat=").u32(battery.mv).str(" mV, System OFF");
  out.event(l);
  radio.powerDown();
  for (int i = 0; i < 3; i++) {
    led.set(RGB_MAGENTA); delay(LOWBAT_BLINK_MS);
    led.set(RGB_OFF);     delay(LOWBAT_BLINK_MS);
  }
  led.shutdown();
  Wire1.end();                                 // TWIM off, but begin()'s pull-ups stay on:
#if defined(PIN_WIRE1_SDA)
  pinMode(PIN_WIRE1_SDA, INPUT);               // no pull, so SDA/SCL can't back-power the
  pinMode(PIN_WIRE1_SCL, INPUT);               // unpowered IMU through its I/O diodes
#endif
#if defined(PIN_LSM6DS3TR_C_POWER)
  digitalWrite(PIN_LSM6DS3TR_C_POWER, LOW);
#endif
  NRF_POWER->SYSTEMOFF = 1;
  for (;;) {}                                  // not reached
}

// Boot check, before anything that could hang or draw much: on battery and
// already low -> back to System OFF (a reset or an unplug wake can't run the
// cell flat). Also takes the first reading reported.
void checkBatteryAtBoot() {
#if defined(PIN_VBAT)
  analogReference(AR_INTERNAL_1_8);            // 0..1.8 V, internal 0.6 V reference
  analogReadResolution(12);
  analogSampleTime(40);                        // us; ~340 kOhm divider source impedance
  delay(5);                                    // P0.31 settles after P0.14 went LOW
  uint16_t mv = readVbatMv();
  bool usb = usbPresent();
  battery.update(millis(), mv, usb);
  if (bootLow(mv, usb)) powerOff("at boot");
#endif
}

void checkBattery(uint32_t t) {
#if defined(PIN_VBAT)
  if (battery.due(t, motorDriven, motorOffAt) && battery.update(t, readVbatMv(), usbPresent()))
    powerOff("cutoff");
#else
  (void)t;
#endif
}

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  led.begin();
  setMotor(false);
#if defined(PIN_CHARGING_CURRENT)
  // BQ25101 charge current: P0.13 LOW = 100 mA (the core leaves it floating =
  // 50 mA). Never drive it HIGH. Matters for the 1800 mAh LiPo: the charger's
  // ~10 h safety timer caps one USB session at ~55 % instead of ~28 %.
  pinMode(PIN_CHARGING_CURRENT, OUTPUT);
  digitalWrite(PIN_CHARGING_CURRENT, LOW);
#endif
#if defined(VBAT_ENABLE)
  // The core drives P0.14 HIGH at boot, which leaves P0.31 (VBAT sense,
  // 1 MΩ / 510 kΩ divider) near 3.6 V, above the pin's limit, mostly while
  // charging. LOW keeps it at ~Vbat/3 (≤1.4 V) and enables reading VBAT on
  // P0.31 (~2.8 µA through the divider). Never drive it HIGH.
  digitalWrite(VBAT_ENABLE, LOW);
#endif
  checkBatteryAtBoot();                        // may not return (System OFF)
  startWatchdog();

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) feedWatchdog();   // give the USB monitor a moment

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
#if defined(NRF52840_XXAA) && defined(USE_TINYUSB)
  // This core otherwise implements micros() with 1024-Hz RTOS ticks (~977 us),
  // coarser than nrf_to_nrf's ~800-us ACK timeout. The checked burst read shifts
  // TX into a phase where that rounded clock can expire before the ACK arrives.
  // Use the core's cycle clock for the radio; application timing stays on millis().
  dwt_enable();
#endif
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
  feedWatchdog();
  keepCrystal();
  out.poll();
  pollSerial();

  unsigned long t = millis();
  if (t - lastSample >= SAMPLE_MS) {
    lastSample = t;
    sample(t);
  }

  if (testRunning(t)) setMotor(true);
  else {
    testUntil = 0;
    setMotor(alert && phaseOn(t, alertStart, PULSE_PERIOD_MS, PULSE_ON_MS));
  }
  checkBattery(millis());                      // motor state is final here; fresh time (sample()'s
                                               // radio.write() may have run past t)
  updateStatusLed(t);

  if (t - lastReport >= 1000) report(t);
}
