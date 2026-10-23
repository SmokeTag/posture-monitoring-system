/*
 * XIAO nRF52840 Sense (chair unit) — the "brain": 8 FSR402s + body IMU, ESB receiver
 * ----------------------------------------------------------------------------------
 * Scans CD74HC4067 channels C8..C15 (one FSR each) on A0, receives the body
 * unit's pitch/roll (firmware/body) over Nordic ESB, calibrates the upright
 * reference, decides the alert and sends it back to the body in the ESB ACK
 * payload (the body drives the motor). Protocol + alert rule:
 * firmware/libraries/PostureLink/PostureLink.h. A laptop on this USB port sees
 * the whole system in one CSV.
 *
 * Libraries: nrf_to_nrf (TMRh20, ESB; runs the RADIO directly, no SoftDevice),
 * PostureLink (firmware/libraries). nrf_to_nrf is polled: the ACK is only sent
 * from inside radio.available(), so loop() must never block (the body retries
 * for ~20 ms before giving up) — hence millis()-scheduled work, no delay().
 *
 * One shared divider (pull-down on SIG; the mux is in the current path):
 *
 *     3V3 ----[ FSR402 ]---- Cn --(mux, R_on ~100R)--+--[ R_FIXED 3.3k ]-- GND
 *                                                    |
 *                                                   SIG --> A0
 *
 *   V = VDD * R_FIXED / (R_FIXED + R_FSR + R_on): rises with force, open FSR
 *   ~0 V. R_on is absorbed by the per-user calibration (see AGENTS.md).
 *
 * The ADC uses reference AR_VDD4 (full scale = VDD), i.e. it is RATIOMETRIC
 * with the divider supply, so R_FSR + R_on = R_FIXED * (ADC_MAX - raw) / raw holds
 * without knowing the exact 3V3 rail voltage.
 *
 * Mux pins: S0=D10  S1=D3  S2=D2  S3=D1  (EN tied to GND, always on).
 * Button: D9 -> GND.  Onboard RGB LED is active LOW.
 * Channels 8..15 => S3 is always HIGH; S2..S0 = the FSR index 0..7.
 *
 * Calibration: sit upright and press the button (D9 -> GND, INPUT_PULLUP,
 * software debounce) or send 'c'. The reference is the mean of the next
 * CAL_SAMPLES body samples (1 s), and so is the FSR baseline (each FSR's mean
 * over the same samples). A capture with the seat rear unloaded (perched
 * forward, empty chair: FsrBaseline.rearOk false) is rejected like an aborted
 * one (previous state kept). 'r' clears both. No alerts until calibrated, and
 * none while the body is silent. 'm' asks the body for a motor test buzz.
 *
 * Motor mute (data captures, so the buzz doesn't steer the sitter): 'q' mutes,
 * 'v' unmutes. CF_MUTE rides in the ChairAck; the body keeps the PWM at 0 on
 * alerts (a motor test still buzzes). The chair decides and logs as usual.
 * Never saved, and cleared when the USB host closes the port (DTR falls), so
 * a capture tool must send 'q' on every connection (live_posture.py
 * --motor-muted does).
 *
 * Alert: IMU + FSR fusion (PostureLink/Fusion.h; docs/decisao-fusao-imu-fsr.md):
 * IMU deviation, slid forward (seat rear unloaded), FSR-confirmed lean; a
 * backward tilt with the hips back (reclined) is accepted and an empty seat
 * never alerts. Spot <-> channel: FSR_SPOT_CH below.
 * Headers are sent on USB connection.
 *
 * Status LED (onboard RGB; colours in PostureLink/StatusLed.h): red fast =
 * radio fault or the body's IMU not delivering (2 s: capture rejected), orange = no body, blue =
 * linked, green fast = capturing the reference, green = calibrated, red
 * slow = posture alert (same 700 ms / 1 s rhythm as the body's motor).
 *
 * Output (prose lines are '#'-prefixed; pandas read_csv(comment='#')), one row
 * per body sample (20 Hz); while the body is silent, one row per FSR scan:
 *   t_ms,cal,calid,dev,alert,dpitch,droll,pitch,roll,boot,seq,
 *   bctl,bcal,bcalid,balert,bmotor,btest,bseq,bage,imu,binh,vbat,c8,...,c15,reason,
 *   mute,bmute
 *   t_ms  = this board's millis() when the row was written (sample received);
 *           never compare it with the body's t_ms
 *   cal / calid / alert = the chair's decision after this sample: reference
 *           valid / its id (new on every capture and clear) / alert. This is
 *           exactly the ChairAck now preloaded, with basis boot/seq of this row
 *   dev   = sqrt(dpitch^2 + droll^2), degrees from upright (0 if uncalibrated)
 *   boot / seq = the body sample (boot id, packet seq); -1 / -1 (IMU fields nan,
 *           b* -1, angles nan) while the body is silent
 *   b*    = what the body reports it was doing when it sent this sample (the
 *           state it applied at sample seq-1; motor/test at send time):
 *     bctl   = 1 chair decides, 2 fallback (body decides alone), 0 none yet
 *     bcal / bcalid = reference held by the body and its id (0 = none)
 *     balert = alert in effect on the body (in bctl 2 it is its own decision)
 *     bmotor = motor output commanded on (not a measurement; muted: would
 *              buzz); btest = a motor-test buzz is running (motor on, but
 *              not a posture alert)
 *     bseq   = the seq of the chair row whose decision the body holds: join
 *              on seq == bseq (same boot); -1 = unknown (none held, or not
 *              made on a sample: chair boot, link timeout, calibration cleared)
 *     bage   = seq - bseq, in body samples: 2 with no loss (decided on N,
 *              applied with the ACK of N+1, reported in N+2); 254 = 254 or
 *              more; -1 = unknown
 *   c8..c15 = FSRs, always raw 12-bit ADC counts (0..4095, open ~0).
 *             Convert to resistance/conductance on the host.
 *   reason = why the alert decision is what it is (Fusion.h Reason): 0 ok,
 *           1 IMU deviation, 2 slumped (slid forward), 3 lean, 4 reclined
 *           (backward tilt accepted), 5 empty seat, 6 invalid IMU; -1 no
 *           decision (not calibrated, capturing, body silent). Since
 *           2026-10-22; older logs end at c15.
 *   mute  = motor muted ('q'): CF_MUTE in the ChairAck now preloaded
 *   bmute = the body reports it holds the mute (BF_MUTE); -1 when body
 *           silent. Both since protocol v5; older logs end at reason.
 *   imu = 0 valid, 1 init, 2 I2C/short read, 3 no new data, 4 invalid vector;
 *         -1 when body silent. Invalid/silent angles and deviations are nan.
 *   binh = body's previous applied state inhibited a chair alert (BF_INH).
 *   vbat = body battery mV, its last reading (0 = none yet; on USB the
 *          charger output); -1 when body silent. Below 3300 on battery the
 *          body powers off (PostureLink/Battery.h): the link then goes silent.
 * Events: "# alert ON/OFF", "# calibrat... calid=N" ("# calibrated upright:"
 * also gives fsr=<baseline c8..c15>), "# calibrate rejected: ...", "# body link up/lost",
 * "# body new session", "# motor muted/unmuted, from ...". The body's boot
 * id (hex) is in "# body link up ...".
 * Stats, normally once per second (counts since the previous stats line; dt =
 * its span in ms, > 1000 only if the USB host stalled):
 *   # rx pkts=N lost=L dup=D old=O rst=R bad=B ver=V sdrop=S edrop=E ble=N dt=T
 *   pkts  = body samples accepted (dup/old excluded)
 *   lost  = seq numbers skipped between packets < LINK_TIMEOUT_MS apart, same
 *           body session. Not the body's noack: a packet can arrive and only
 *           its ACK get lost. Outages and body reboots are not counted here.
 *   dup / old = repeated / out-of-order packets, ignored (they don't keep the
 *           link up: "# body link lost" comes 1 s after the last NEW sample)
 *   rst   = body restarts seen (boot id changed, or seq went back/jumped;
 *           best effort — see SeqTracker in PostureLink.h)
 *   bad   = malformed/incoherent payloads; ver = incompatible format (flash both boards)
 *   sdrop / edrop = CSV rows / '#' events dropped because the USB host was not
 *           reading (firmware/libraries/PostureLink/SerialOut.h); the header is
 *           re-sent whenever a host opens the port.
 *   ble   = BLE beacons sent (below)
 *
 * Wireless monitor (PostureLink/Beacon.h + BleAdv.h): ~20 times a second,
 * right after a body sample, the RADIO briefly switches to BLE and sends the
 * chair's state as a non-connectable advertisement (~1.3 ms), then goes back
 * to ESB; analysis/ble_chair.py reads it (mostra.py --ble). One-way: with no
 * USB host the chair clears the calibration itself when the seat stays empty
 * SEAT_EMPTY_CLEAR_MS (the end of a visit), since no host can send 'r'.
 *
 * Build / upload / monitor (arduino-cli, run from the repo root):
 *   arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense --libraries firmware/libraries firmware/chair
 *   CHAIR=$(readlink -f /dev/serial/by-id/usb-Seeed_XIAO_nRF52840_Sense_2C7FE46C8E63BDA0-if00)
 *   arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p "$CHAIR" firmware/chair
 *   arduino-cli monitor -p "$CHAIR" -c baudrate=115200
 */

#include <Adafruit_TinyUSB.h>                  // needed for USB Serial on this core
#include <nrf_to_nrf.h>
#include <PostureLink.h>
#include <Fusion.h>
#include <SerialOut.h>
#include <RandomId.h>
#include <StatusLed.h>
#include <Beacon.h>
#include <BleAdv.h>

using namespace posture;

const int   SIG_PIN    = A0;
const int   SEL_PINS[4] = {D10, D3, D2, D1};   // S0, S1, S2, S3
const int   FIRST_CH    = 8;                   // FSRs on C8..C15
const int   N_FSR       = 8;

const int   SETTLE_US   = 50;                  // mux switch + SIG node settling
const unsigned long SCAN_MS = 50;              // 20 Hz FSR scan rate

const int   CAL_SAMPLES = 20;                  // 1 s of body samples averaged into the reference

// FSR channel (C8..C15) at each chair spot, Spot order (Fusion.h). Same as
// analysis/chair_map.json (sitter's left/right); the chair wiring is fixed,
// so re-mapping the chair means updating this table too.
const uint8_t FSR_SPOT_CH[N_SPOTS] = {
  8,   // seat_front_left
  9,   // seat_front_right
  10,  // seat_rear_left
  11,  // seat_rear_right
  14,  // back_lower_left
  12,  // back_lower_right
  15,  // back_upper_left
  13,  // back_upper_right
};

const int   BTN_PIN     = D9;                  // calibrate button to GND
const int   DEBOUNCE_MS = 25;                  // input must be stable this long
const unsigned long PULSE_PERIOD_MS = 1000, PULSE_ON_MS = 700;   // alert LED, as the body's motor
const unsigned long IMU_FAULT_MS = 250;        // body IMU invalid over this span of samples = fault
const unsigned long REJECT_LED_MS = 2000;      // rejected capture: red fast this long
const unsigned long SEAT_EMPTY_CLEAR_MS = 10000;  // no USB host: seat empty this long clears
                                               // the calibration (end of an exhibit visit)

nrf_to_nrf radio;
StatusLed led(LED_RED, LED_GREEN, LED_BLUE, 255);   // USB powered: full brightness
int  fsr[N_FSR] = {};                          // last scan, raw ADC counts
unsigned long lastScan = 0;

// body link
BodyPacket body;
bool bodyUp = false;
unsigned long lastBodyAt = 0;
unsigned long imuOkAt = 0;                     // last body sample with a valid IMU
SeqTracker seqs;                               // pkts/lost/dup/old/rst, reported once per second
uint32_t versionPkts = 0;
bool versionNoted = false;
int lastImuState = -1;
uint32_t badPkts = 0;                          // rejected packets, since the last stats line
unsigned long lastReport = 0;
ChairAck ack = {};                             // preloaded as the ESB ACK payload
bool radioFailed = false;                      // reported in the header
bool muted = false;                            // CF_MUTE: motor muted for a data capture
bool hostWas = false;                          // USB host connected at the last loop()

// BLE beacon (Beacon.h, BleAdv.h): the live state for the wireless monitor
uint8_t bleAddr[6];
uint8_t beaconSeq = 0;
unsigned long lastBeacon = 0;
uint32_t beacons = 0;                          // sent, since the last stats line
bool beaconFailNoted = false;
unsigned long seatEmptySince = 0;
bool seatEmpty = false;

bool chairHeader(uint8_t i, Line &l);
SerialOut<decltype(Serial)> out(Serial, chairHeader);

// calibration
bool  calibrated = false;
int   calLeft = 0;                             // >0 while capturing the reference
float calPSum = 0, calRSum = 0;
float pitchRef = 0, rollRef = 0;
float calFsrSum[N_FSR] = {};                   // FSR baseline capture, by channel
float fsrBase[N_FSR] = {};                     // FSR baseline (raw means), by channel
FsrBaseline fusionBase;
uint8_t calId = 0;                             // ChairAck.calId: random at boot, new on every capture/clear

FusionGate gate;
int reason = -1;                               // Reason of the last decision; -1 = none
unsigned long rejectedAt = 0;                  // last capture rejected (LED: red fast, REJECT_LED_MS)
bool rejectShown = false;
unsigned long alertAt = 0;                     // alert ON time: LED pulse phase

bool btnRaw = HIGH, btnState = HIGH;           // last read / debounced level
unsigned long btnChangedAt = 0;

void selectChannel(int ch) {
  for (int b = 0; b < 4; b++) digitalWrite(SEL_PINS[b], (ch >> b) & 1);
}

int readChannel(int ch) {
  selectChannel(ch);
  delayMicroseconds(SETTLE_US);
  analogRead(SIG_PIN);                         // discard: first conversion after switching
  int raw = analogRead(SIG_PIN);
  return raw < 0 ? 0 : raw;
}

bool chairHeader(uint8_t i, Line &l) {
  switch (i) {
    case 0: l.str("# chair unit (brain): FSRs C8..C15 + body IMU. 'c'/button = calibrate, "
                  "'r' = clear, 'm' = motor test, 'q'/'v' = motor mute/unmute"); return true;
    case 1: if (radioFailed) l.str("# radio init FAILED"); return true;
    case 2: l.str("# unit=raw"); return true;
    case 4: l.str("# proto=").u32(PROTO_VER); return true;
    case 3:
      l.str("t_ms,cal,calid,dev,alert,dpitch,droll,pitch,roll,boot,seq,"
            "bctl,bcal,bcalid,balert,bmotor,btest,bseq,bage,imu,binh,vbat");
      for (int c = 0; c < N_FSR; c++) l.str(",c").u32(FIRST_CH + c);
      l.str(",reason,mute,bmute");
      return true;
  }
  return false;
}

void event(const char *s) { Line l; l.str(s); out.event(l); }

void updateStatusLed(unsigned long t) {
  bool fast = phaseOn(t, 0, 2 * BLINK_FAST_MS, BLINK_FAST_MS);
  // IMU fault = invalid across received samples; a silent radio is not an IMU fault
  bool imuFault = bodyUp && lastBodyAt - imuOkAt >= IMU_FAULT_MS;
  if (rejectShown && t - rejectedAt >= REJECT_LED_MS) rejectShown = false;
  if (radioFailed || imuFault || rejectShown) led.set(fast ? RGB_RED : RGB_OFF);
  else if (!bodyUp) led.set(RGB_ORANGE);
  else if (calLeft > 0) led.set(fast ? RGB_GREEN : RGB_OFF);
  else if (gate.alert) led.set(phaseOn(t, alertAt, PULSE_PERIOD_MS, PULSE_ON_MS) ? RGB_RED : RGB_OFF);
  else led.set(calibrated ? RGB_GREEN : RGB_BLUE);
}

// Preload the ACK payload: the body gets it with its next packet.
void writeAck() { radio.writeAckPayload(RF_PIPE, &ack, sizeof(ack)); }

// New decision state into the ACK. onSample: made on the body sample in `body`
// (its basis); otherwise (boot, link timeout, calibration cleared) no basis.
void updateAck(bool onSample) {
  ack.ver = PROTO_VER;
  ack.flags = (calibrated ? CF_CAL : 0) | (gate.alert ? CF_ALERT : 0) | (muted ? CF_MUTE : 0);
  ack.calId = calId;
  ack.pitchRef = pitchRef;
  ack.rollRef = rollRef;
  ack.basisBoot = onSample ? body.boot : 0;
  ack.basisSeq = onSample ? body.seq : 0;
  writeAck();
}

// Same decision (and basis), mute bit changed. Re-sent ('q' on every
// connection) it only logs again.
void setMute(bool on, const char *src) {
  muted = on;
  ack.flags = (ack.flags & ~CF_MUTE) | (on ? CF_MUTE : 0);
  writeAck();
  Line l;
  l.str(on ? "# motor muted (alerts still decided and logged), from " : "# motor unmuted, from ")
   .str(src);
  out.event(l);
}

void newCalId() { calId = calId == 255 ? 1 : calId + 1; }

void setAlert(bool on, float dev) {
  gate.alert = on;
  if (on) alertAt = millis();
  Line l;
  l.str(on ? "# alert ON  dev=" : "# alert OFF dev=").fix(dev, 1);
  out.event(l);
}

void startCalibration(const char *src) {
  Line l;
  if (!bodyUp || body.imu != IMU_OK) {
    l.str("# calibrate ignored (no valid IMU), from ").str(src);
    out.event(l);
    return;
  }
  calLeft = CAL_SAMPLES;
  calPSum = calRSum = 0;
  for (int i = 0; i < N_FSR; i++) calFsrSum[i] = 0;
  l.str("# calibrating (hold upright 1 s), from ").str(src);
  out.event(l);
}

void clearCalibration() {
  calibrated = false; calLeft = 0;
  for (int i = 0; i < N_FSR; i++) fsrBase[i] = 0;
  fusionBase = FsrBaseline();
  newCalId();
  if (gate.alert) setAlert(false, 0);
  gate.reset();
  updateAck(false);
  Line l;
  l.str("# calibration cleared calid=").u32(calId);
  out.event(l);
}

void pollButton() {
  bool r = digitalRead(BTN_PIN);
  unsigned long now = millis();
  if (r != btnRaw) { btnRaw = r; btnChangedAt = now; }
  else if (r != btnState && now - btnChangedAt >= DEBOUNCE_MS) {
    btnState = r;
    if (btnState == LOW) startCalibration("button");   // act on press, not release
  }
}

void pollSerial() {
  for (int n = 0; n < 16 && Serial.available() > 0; n++) {   // bounded per loop()
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      startCalibration("serial");
    } else if (c == 'r' || c == 'R') {
      clearCalibration();
    } else if (c == 'm' || c == 'M') {
      if (!bodyUp) { event("# motor test ignored (no body data)"); continue; }
      ack.buzz++;                              // same decision (and basis), new request
      writeAck();
      event("# motor test requested");
    } else if (c == 'q' || c == 'Q') {
      setMute(true, "serial");
    } else if (c == 'v' || c == 'V') {
      setMute(false, "serial");
    }
  }
}

// t_ms,cal,calid,dev,alert,dpitch,droll,pitch,roll,boot,seq,
// bctl,bcal,bcalid,balert,bmotor,btest,bseq,bage,imu,binh,vbat,c8..c15,reason,mute,bmute
void printRow(unsigned long t, float dev, float dPitch, float dRoll) {
  Line l;
  l.u32(t).chr(',').u32(calibrated).chr(',').u32(calId).chr(',').fix(dev, 1).chr(',')
   .u32(gate.alert).chr(',').fix(dPitch, 1).chr(',').fix(dRoll, 1).chr(',')
   .fix(bodyUp ? body.pitch : NAN, 1).chr(',').fix(bodyUp ? body.roll : NAN, 1);
  if (bodyUp) {
    bool known = body.age != AGE_NONE;
    l.chr(',').u32(body.boot).chr(',').u32(body.seq).chr(',').u32(body.ctl)
     .chr(',').u32(!!(body.flags & BF_CAL)).chr(',').u32(body.calId)
     .chr(',').u32(!!(body.flags & BF_ALERT)).chr(',').u32(!!(body.flags & BF_MOTOR))
     .chr(',').u32(!!(body.flags & BF_TEST))
     .chr(',').i32(known ? (int32_t)body.ackSeq : -1).chr(',').i32(known ? (int32_t)body.age : -1);
  } else {
    for (int i = 0; i < 10; i++) l.str(",-1");   // boot,seq,bctl..bage: no sample
  }
  l.chr(',').i32(bodyUp ? body.imu : -1).chr(',')
   .i32(bodyUp ? !!(body.flags & BF_INH) : -1).chr(',').i32(bodyUp ? body.vbat : -1);
  for (int i = 0; i < N_FSR; i++) l.chr(',').i32(fsr[i]);
  l.chr(',').i32(reason).chr(',').u32(muted)
   .chr(',').i32(bodyUp ? !!(body.flags & BF_MUTE) : -1);
  out.row(l);
}

// Fusion decision (Fusion.h) on this sample + the FSRs of the last scan.
void decideAlert(float dPitch, float dRoll, float dev, unsigned long t) {
  float spot[N_SPOTS];
  for (int s = 0; s < N_SPOTS; s++) spot[s] = fsr[FSR_SPOT_CH[s] - FIRST_CH];
  Decision d = decide(dPitch, dRoll, spot, fusionBase);
  reason = d.reason;
  if (d.reason == R_EMPTY || d.reason == R_INVALID) {   // no-one seated: no alert, hold restarts
    if (gate.alert) setAlert(false, dev);
    gate.reset();
  } else if (gate.update(d.bad, d.ok, t)) {
    setAlert(gate.alert, dev);
  }
}

// End of a capture: IMU reference + FSR baseline, unless the seat rear
// carried no load (perched forward or an empty chair: no slump detection
// with that baseline). A rejected capture is like an aborted one: the
// previous state (reference or none) stays.
void finishCapture(unsigned long t) {
  float base[N_FSR], spot[N_SPOTS];
  for (int i = 0; i < N_FSR; i++) base[i] = calFsrSum[i] / CAL_SAMPLES;
  for (int s = 0; s < N_SPOTS; s++) spot[s] = base[FSR_SPOT_CH[s] - FIRST_CH];
  FsrBaseline fb = FsrBaseline::of(spot);
  if (!fb.rearOk) {
    rejectedAt = t;
    rejectShown = true;
    Line l;
    l.str("# calibrate rejected: seat rear unloaded (sit back fully and recalibrate) rear=")
     .fix(fb.rear, 0);
    out.event(l);
    return;
  }
  pitchRef = calPSum / CAL_SAMPLES;
  rollRef  = calRSum / CAL_SAMPLES;
  for (int i = 0; i < N_FSR; i++) fsrBase[i] = base[i];
  fusionBase = fb;
  calibrated = true;
  newCalId();
  if (gate.alert) setAlert(false, 0);
  gate.reset();
  Line l;
  l.str("# calibrated upright: pitch=").fix(pitchRef, 1).str(" roll=").fix(rollRef, 1)
   .str(" calid=").u32(calId).str(" fsr=");
  for (int i = 0; i < N_FSR; i++) { if (i) l.chr(','); l.fix(fsrBase[i], 1); }
  out.event(l);
}

// One body sample: calibration capture, alert decision, ACK, CSV row.
void onBodySample(unsigned long t) {
  if (body.imu != lastImuState) {
    Line e; e.str("# body IMU state=").u32(body.imu); out.event(e);
    lastImuState = body.imu;
  }
  if (body.imu != IMU_OK) {
    if (calLeft) event("# calibration aborted (invalid IMU)");
    calLeft = 0;
    if (gate.alert) setAlert(false, NAN);
    gate.reset();
    reason = calibrated ? R_INVALID : -1;
    updateAck(false);                         // health heartbeat is not a posture decision
    printRow(t, NAN, NAN, NAN);
    return;
  }
  imuOkAt = t;
  if (calLeft > 0) {
    calPSum += body.pitch; calRSum += body.roll;
    for (int i = 0; i < N_FSR; i++) calFsrSum[i] += fsr[i];
    if (--calLeft == 0) finishCapture(t);
  }

  float dPitch = body.pitch - pitchRef, dRoll = body.roll - rollRef;
  float dev = calibrated ? deviation(body.pitch, body.roll, pitchRef, rollRef) : 0;
  reason = -1;
  if (calibrated && calLeft == 0) decideAlert(dPitch, dRoll, dev, t);

  updateAck(true);
  printRow(t, dev, dPitch, dRoll);
}

void checkBodyLink(unsigned long t) {
  if (!bodyUp || t - lastBodyAt < LINK_TIMEOUT_MS) return;
  bodyUp = false;
  calLeft = 0;                                 // an interrupted capture is discarded
  if (gate.alert) setAlert(false, 0);
  gate.reset();
  updateAck(false);
  event("# body link lost");
}

// True when a new body sample was handled (its ACK has gone out).
bool pollRadio() {
  if (!radio.available()) return false;        // also sends the ACK (+ payload)
  uint8_t buf[32];                             // = the DPL max payload
  uint8_t len = radio.getDynamicPayloadSize();
  if (len > sizeof(buf)) len = sizeof(buf);
  radio.read(buf, len);
  BodyPacket pkt;
  PacketStatus status = classifyBodyPacket(buf, len, pkt);
  if (status == PACKET_VERSION) {
    versionPkts++;
    if (!versionNoted) {
      versionNoted = true;
      Line e; e.str("# body protocol v").u32(payloadVersion(buf, len))
               .str(" != v").u32(PROTO_VER).str(": flash both boards"); out.event(e);
    }
    return false;
  }
  if (status != PACKET_OK) { badPkts++; return false; }   // not a sample at all

  unsigned long t = millis();
  checkBodyLink(t);                            // a timed-out link is declared lost (alert
                                               // cleared, capture aborted) before this packet
  SeqKind k = seqs.onPacket(pkt.boot, pkt.seq, t);
  // Not a new sample: it doesn't keep the link up either, or a stream of them
  // would hold bodyUp (and a stale alert in the ACK) without fresh decisions.
  if (k == SEQ_DUP || k == SEQ_OLD) return false;
  lastBodyAt = t;
  body = pkt;

  bool fresh = k == SEQ_FIRST || k == SEQ_RESTART;
  if (!bodyUp || k == SEQ_RESTART) {
    Line l;
    l.str(!bodyUp ? "# body link up boot=" : "# body new session boot=").hex8(pkt.boot);
    if (!bodyUp) l.str(fresh ? " new" : " resumed");
    out.event(l);
    bodyUp = true;
  }
  onBodySample(t);
  return true;
}

// The chair's state as one BLE advertisement (Beacon.h). Sent right after a
// body sample (the next one is ~50 ms away), or on the timer while the body
// is silent; never with an ESB packet pending.
void sendBeacon(unsigned long t) {
  BeaconState b;
  b.seq = beaconSeq++;
  b.t = t;
  b.flags = (calibrated ? BK_CAL : 0) | (gate.alert ? BK_ALERT : 0) | (bodyUp ? BK_BODY : 0) |
            (calLeft > 0 ? BK_CAPTURE : 0) | (muted ? BK_MUTE : 0) | (radioFailed ? BK_RADIO : 0) |
            (rejectShown ? BK_REJECT : 0) | (out.connected() ? BK_USB : 0);
  b.calId = calId;
  b.reason = (int8_t)reason;
  for (int i = 0; i < N_FSR; i++) b.fsr[i] = (uint16_t)fsr[i];
  if (bodyUp) {
    b.bflags = (body.ctl & 3) | (body.flags & BF_ALERT ? BB_ALERT : 0) |
               (body.flags & BF_MOTOR ? BB_MOTOR : 0) | (body.flags & BF_TEST ? BB_TEST : 0) |
               (body.flags & BF_INH ? BB_INH : 0) | (body.flags & BF_MUTE ? BB_MUTE : 0) |
               (body.flags & BF_CAL ? BB_CAL : 0);
    b.imu = (int8_t)body.imu;
    b.vbat = body.vbat;
    if (body.imu == IMU_OK) {
      b.dPitch = body.pitch - pitchRef;
      b.dRoll = body.roll - rollRef;
    }
  }
  uint8_t pdu[ADV_PDU_LEN];
  advPdu(bleAddr, b, pdu);
  bool ok = bleAdvertise(pdu);
  radio.startListening();                      // back to ESB RX
  lastBeacon = t;
  if (ok) beacons++;
  else if (!beaconFailNoted) { beaconFailNoted = true; event("# ble beacon: RADIO timeout"); }
}

// No USB host (the exhibit runs on the beacon): when the seat stays empty
// SEAT_EMPTY_CLEAR_MS the visitor has left, so the next one isn't buzzed
// with their reference. With a host (captures, live_posture.py) nothing
// changes: the host clears ('r') when it wants to.
void autoClear(unsigned long t) {
  float seat = 0;
  for (int s = SEAT_FRONT_LEFT; s <= SEAT_REAR_RIGHT; s++) seat += fsr[FSR_SPOT_CH[s] - FIRST_CH];
  bool empty = seat < EMPTY_RAW;
  if (empty && !seatEmpty) seatEmptySince = t;
  seatEmpty = empty;
  if (!empty || !calibrated || out.connected() || t - seatEmptySince < SEAT_EMPTY_CLEAR_MS) return;
  event("# seat empty, no USB host: clearing the calibration");
  clearCalibration();
}

// A full scan takes ~2 ms, longer than the body waits for an ACK (~0.8 ms);
// both run at 20 Hz, so their phases drift slowly and packets would hit the
// scan in long bursts. available() between channels sends the ACK in time;
// the packet stays pending until pollRadio() reads it.
void scanFsrs() {
  for (int i = 0; i < N_FSR; i++) {
    fsr[i] = readChannel(FIRST_CH + i);
    radio.available();
  }
}

// TinyUSB stops the 16 MHz crystal when USB is unplugged (dcd_nrf5x.c,
// USB_EVT_REMOVED -> hfclk_disable); nrf_to_nrf starts it only in begin().
// On the internal RC clock the radio is off frequency: unplugging USB while
// on the LiPo would drop the link until a reset. Restart it whenever it stops
// (same as body.ino).
void keepCrystal() {
#if defined(NRF52840_XXAA)
  const uint32_t xtalOn = CLOCK_HFCLKSTAT_SRC_Msk | CLOCK_HFCLKSTAT_STATE_Msk;
  if ((NRF_CLOCK->HFCLKSTAT & xtalOn) != xtalOn) NRF_CLOCK->TASKS_HFCLKSTART = 1;
#endif
}

void setup() {
#if defined(PIN_CHARGING_CURRENT)
  // BQ25101 charge current: P0.13 LOW = 100 mA (the core leaves it floating =
  // 50 mA). Never drive it HIGH. Same 1800 mAh LiPo as the body: the charger's
  // ~10 h safety timer caps one USB session at ~55 % instead of ~28 %.
  pinMode(PIN_CHARGING_CURRENT, OUTPUT);
  digitalWrite(PIN_CHARGING_CURRENT, LOW);
#endif
#if defined(VBAT_ENABLE)
  // Same LiPo circuit as the body: the core drives P0.14 HIGH at boot, which
  // leaves P0.31 (VBAT sense, 1 MΩ / 510 kΩ divider) near 3.6 V, above the
  // pin's limit, mostly while charging. LOW keeps it at ~Vbat/3 (≤1.4 V).
  // Never drive it HIGH.
  digitalWrite(VBAT_ENABLE, LOW);
#endif
  Serial.begin(115200);
  for (int b = 0; b < 4; b++) pinMode(SEL_PINS[b], OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  led.begin();
  analogReference(AR_VDD4);                    // full scale = VDD -> ratiometric
  analogReadResolution(12);
  analogSampleTime(10);                        // us; source impedance <= 3.3 kOhm divider

  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // give the USB monitor a moment

  radioFailed = !radio.begin();
  radio.setChannel(RF_CHANNEL);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openReadingPipe(RF_PIPE, RF_ADDR);
  calId = randomId();                          // a chair reboot shouldn't reuse old ids
  bleAddress(bleAddr);
  updateAck(false);                            // preload before the first packet
  radio.startListening();                      // RX role

  out.poll();                                  // header goes out once a host is there
}

void report(unsigned long t) {
  Line l;
  l.str("# rx pkts=").u32(seqs.pkts).str(" lost=").u32(seqs.lost)
   .str(" dup=").u32(seqs.dup).str(" old=").u32(seqs.old).str(" rst=").u32(seqs.rst)
   .str(" bad=").u32(badPkts).str(" ver=").u32(versionPkts)
   .str(" sdrop=").u32(out.rowDrops()).str(" edrop=").u32(out.eventDrops())
   .str(" ble=").u32(beacons).str(" dt=").u32(t - lastReport);
  if (!out.stats(l)) return;                   // host not reading: keep counting
  lastReport = t;
  seqs.clearCounts();
  badPkts = versionPkts = beacons = 0;
  out.clearDrops();
}

void loop() {
  keepCrystal();
  bool sampled = pollRadio();
  out.poll();
  // The capture tool went away: back to the safe default (motor on).
  if (hostWas && !out.connected() && muted) setMute(false, "host disconnected");
  hostWas = out.connected();
  pollSerial();
  pollButton();

  unsigned long t = millis();
  checkBodyLink(t);
  updateStatusLed(t);
  if (t - lastScan >= SCAN_MS) {
    lastScan = t;
    scanFsrs();
    autoClear(t);
    if (!bodyUp) { reason = -1; printRow(t, NAN, NAN, NAN); }
  }
  if ((sampled || t - lastBeacon >= 2 * BEACON_MS ||
       (!bodyUp && t - lastBeacon >= BEACON_MS)) && !radio.available())
    sendBeacon(t);

  if (t - lastReport >= 1000) report(t);
}
