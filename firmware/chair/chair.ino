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
 * Per-channel divider (pull-down; mux sits on the SENSE side only):
 *
 *     3V3 ----[ FSR402 ]----+----[ R_FIXED 3.3k ]---- GND
 *                           |
 *                          Cn --(mux)--> SIG --> A0
 *
 *   V = VDD * R_FIXED / (R_FIXED + R_FSR): rises with force, open FSR ~0 V.
 *
 * The ADC uses reference AR_VDD4 (full scale = VDD), i.e. it is RATIOMETRIC
 * with the divider supply, so R_FSR = R_FIXED * (ADC_MAX - raw) / raw holds
 * without knowing the exact 3V3 rail voltage.
 *
 * Mux pins: S0=D10  S1=D3  S2=D2  S3=D1  (EN tied to GND, always on).
 * Button: D9 -> GND.  Onboard RGB LED is active LOW.
 * Channels 8..15 => S3 is always HIGH; S2..S0 = the FSR index 0..7.
 *
 * Calibration: sit upright and press the button (D9 -> GND, INPUT_PULLUP,
 * software debounce) or send 'c'. The reference is the mean of the next
 * CAL_SAMPLES body samples (1 s); the green LED flashes when it is captured.
 * 'r' clears it. No alerts until calibrated, and none while the body is silent.
 * 'm' asks the body for a motor test buzz. 'u' toggles the FSR unit.
 *
 * Output (prose lines are '#'-prefixed; pandas read_csv(comment='#')), one row
 * per body sample (20 Hz); while the body is silent, one row per FSR scan:
 *   t_ms,cal,calid,dev,alert,dpitch,droll,pitch,roll,boot,seq,
 *   bctl,bcal,bcalid,balert,bmotor,btest,bseq,bage,c8,...,c15
 *   t_ms  = this board's millis() when the row was written (sample received);
 *           never compare it with the body's t_ms
 *   cal / calid / alert = the chair's decision after this sample: reference
 *           valid / its id (new on every capture and clear) / alert. This is
 *           exactly the ChairAck now preloaded, with basis boot/seq of this row
 *   dev   = sqrt(dpitch^2 + droll^2), degrees from upright (0 if uncalibrated)
 *   boot / seq = the body sample (boot id, packet seq); -1 / -1 (IMU fields 0,
 *           b* -1) while the body is silent
 *   b*    = what the body reports it was doing when it sent this sample (the
 *           state it applied at sample seq-1; motor/test at send time):
 *     bctl   = 1 chair decides, 2 fallback (body decides alone), 0 none yet
 *     bcal / bcalid = reference held by the body and its id (0 = none)
 *     balert = alert in effect on the body (in bctl 2 it is its own decision)
 *     bmotor = motor output commanded on (not a measurement); btest = a
 *              motor-test buzz is running (motor on, but not a posture alert)
 *     bseq   = the seq of the chair row whose decision the body holds: join
 *              on seq == bseq (same boot); -1 = unknown (none held, or not
 *              made on a sample: chair boot, link timeout, calibration cleared)
 *     bage   = seq - bseq, in body samples: 2 with no loss (decided on N,
 *              applied with the ACK of N+1, reported in N+2); 254 = 254 or
 *              more; -1 = unknown
 *   c8..c15 = FSRs, ohms (-1 = open) or raw ADC counts ('u')
 * Events: "# alert ON/OFF", "# calibrat... calid=N", "# body link up/lost",
 * "# body new session". The body's boot id (hex) is in "# body link up ...".
 * Stats, normally once per second (counts since the previous stats line; dt =
 * its span in ms, > 1000 only if the USB host stalled):
 *   # rx pkts=N lost=L dup=D old=O rst=R bad=B sdrop=S edrop=E dt=T
 *   pkts  = body samples accepted (dup/old excluded)
 *   lost  = seq numbers skipped between packets < LINK_TIMEOUT_MS apart, same
 *           body session. Not the body's noack: a packet can arrive and only
 *           its ACK get lost. Outages and body reboots are not counted here.
 *   dup / old = repeated / out-of-order packets, ignored (they don't keep the
 *           link up: "# body link lost" comes 1 s after the last NEW sample)
 *   rst   = body restarts seen (boot id changed, or seq went back/jumped;
 *           best effort — see SeqTracker in PostureLink.h)
 *   bad   = packets rejected: other protocol version/size (a body running
 *           older firmware: flash both boards) or incoherent contents
 *   sdrop / edrop = CSV rows / '#' events dropped because the USB host was not
 *           reading (firmware/libraries/PostureLink/SerialOut.h); the header is
 *           re-sent whenever a host opens the port.
 *
 * Build / upload / monitor (arduino-cli, run from the repo root):
 *   arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense --libraries firmware/libraries firmware/chair
 *   arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p /dev/ttyACM0 firmware/chair
 *   arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
 */

#include <Adafruit_TinyUSB.h>                  // needed for USB Serial on this core
#include <nrf_to_nrf.h>
#include <PostureLink.h>
#include <SerialOut.h>
#include <RandomId.h>

using namespace posture;

const int   SIG_PIN    = A0;
const int   SEL_PINS[4] = {D10, D3, D2, D1};   // S0, S1, S2, S3
const int   FIRST_CH    = 8;                   // FSRs on C8..C15
const int   N_FSR       = 8;

const float R_FIXED     = 3300.0;              // ohms (nominal; use measured if known)
const int   ADC_MAX     = 4095;                // 12-bit
const int   OPEN_RAW    = 8;                   // below this => open / no force (~6 mV)
const int   SETTLE_US   = 50;                  // mux switch + SIG node settling
const unsigned long SCAN_MS = 50;              // 20 Hz FSR scan rate

const int   CAL_SAMPLES = 20;                  // 1 s of body samples averaged into the reference

const int   BTN_PIN     = D9;                  // calibrate button to GND
const int   DEBOUNCE_MS = 25;                  // input must be stable this long
const int   CAL_LED     = LED_GREEN;           // flashes when the reference is captured
const int   LED_ON      = LOW;                 // RGB LED is active LOW (core's LED_STATE_ON is wrong)
const int   FLASH_MS    = 300;

nrf_to_nrf radio;
bool rawMode = false;                          // 'u' toggles ohms <-> raw counts
int  fsr[N_FSR] = {};                          // last scan, raw ADC counts
unsigned long lastScan = 0;

// body link
BodyPacket body;
bool bodyUp = false;
unsigned long lastBodyAt = 0;
SeqTracker seqs;                               // pkts/lost/dup/old/rst, reported once per second
uint32_t badPkts = 0;                          // rejected packets, since the last stats line
unsigned long lastReport = 0;
ChairAck ack = {};                             // preloaded as the ESB ACK payload
bool radioFailed = false;                      // reported in the header

bool chairHeader(uint8_t i, Line &l);
SerialOut<decltype(Serial)> out(Serial, chairHeader);
const uint8_t HDR_UNIT = 2;                    // chairHeader(): first line re-sent by 'u'

// calibration
bool  calibrated = false;
int   calLeft = 0;                             // >0 while capturing the reference
float calPSum = 0, calRSum = 0;
float pitchRef = 0, rollRef = 0;
uint8_t calId = 0;                             // ChairAck.calId: random at boot, new on every capture/clear

AlertGate gate;

bool btnRaw = HIGH, btnState = HIGH;           // last read / debounced level
unsigned long btnChangedAt = 0;
unsigned long ledOnAt = 0;
bool ledLit = false;

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

float rawToOhms(int raw) {
  // raw/ADC_MAX = R_FIXED / (R_FIXED + R_FSR)  =>  R_FSR = R_FIXED * (ADC_MAX - raw) / raw
  return (raw >= OPEN_RAW) ? R_FIXED * (ADC_MAX - raw) / raw : -1.0;
}

bool chairHeader(uint8_t i, Line &l) {
  switch (i) {
    case 0: l.str("# chair unit (brain): FSRs C8..C15 + body IMU. 'c'/button = calibrate, "
                  "'r' = clear, 'm' = motor test, 'u' = toggle ohms/raw"); return true;
    case 1: if (radioFailed) l.str("# radio init FAILED"); return true;
    case HDR_UNIT: l.str(rawMode ? "# unit=raw" : "# unit=ohms (-1 = open)"); return true;
    case HDR_UNIT + 1:
      l.str("t_ms,cal,calid,dev,alert,dpitch,droll,pitch,roll,boot,seq,"
            "bctl,bcal,bcalid,balert,bmotor,btest,bseq,bage");
      for (int c = 0; c < N_FSR; c++) l.str(",c").u32(FIRST_CH + c);
      return true;
  }
  return false;
}

void event(const char *s) { Line l; l.str(s); out.event(l); }

void flashLed() {
  digitalWrite(CAL_LED, LED_ON);
  ledLit = true;
  ledOnAt = millis();
}

void updateLed() {
  if (ledLit && millis() - ledOnAt >= FLASH_MS) { digitalWrite(CAL_LED, !LED_ON); ledLit = false; }
}

// Preload the ACK payload: the body gets it with its next packet.
void writeAck() { radio.writeAckPayload(RF_PIPE, &ack, sizeof(ack)); }

// New decision state into the ACK. onSample: made on the body sample in `body`
// (its basis); otherwise (boot, link timeout, calibration cleared) no basis.
void updateAck(bool onSample) {
  ack.ver = PROTO_VER;
  ack.flags = (calibrated ? CF_CAL : 0) | (gate.alert ? CF_ALERT : 0);
  ack.calId = calId;
  ack.pitchRef = pitchRef;
  ack.rollRef = rollRef;
  ack.basisBoot = onSample ? body.boot : 0;
  ack.basisSeq = onSample ? body.seq : 0;
  writeAck();
}

void newCalId() { calId = calId == 255 ? 1 : calId + 1; }

void setAlert(bool on, float dev) {
  gate.alert = on;
  Line l;
  l.str(on ? "# alert ON  dev=" : "# alert OFF dev=").fix(dev, 1);
  out.event(l);
}

void startCalibration(const char *src) {
  Line l;
  if (!bodyUp) {
    l.str("# calibrate ignored (no body data), from ").str(src);
    out.event(l);
    return;
  }
  calLeft = CAL_SAMPLES;
  calPSum = calRSum = 0;
  l.str("# calibrating (hold upright 1 s), from ").str(src);
  out.event(l);
}

void clearCalibration() {
  calibrated = false; calLeft = 0;
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
    if (c == 'u' || c == 'U') {
      rawMode = !rawMode;
      out.header(HDR_UNIT);
    } else if (c == 'c' || c == 'C') {
      startCalibration("serial");
    } else if (c == 'r' || c == 'R') {
      clearCalibration();
    } else if (c == 'm' || c == 'M') {
      if (!bodyUp) { event("# motor test ignored (no body data)"); continue; }
      ack.buzz++;                              // same decision (and basis), new request
      writeAck();
      event("# motor test requested");
    }
  }
}

// t_ms,cal,calid,dev,alert,dpitch,droll,pitch,roll,boot,seq,
// bctl,bcal,bcalid,balert,bmotor,btest,bseq,bage,c8..c15
void printRow(unsigned long t, float dev, float dPitch, float dRoll) {
  Line l;
  l.u32(t).chr(',').u32(calibrated).chr(',').u32(calId).chr(',').fix(dev, 1).chr(',')
   .u32(gate.alert).chr(',').fix(dPitch, 1).chr(',').fix(dRoll, 1).chr(',')
   .fix(bodyUp ? body.pitch : 0, 1).chr(',').fix(bodyUp ? body.roll : 0, 1);
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
  for (int i = 0; i < N_FSR; i++) {
    l.chr(',');
    if (rawMode) l.i32(fsr[i]);
    else         l.fix(rawToOhms(fsr[i]), 0);
  }
  out.row(l);
}

// One body sample: calibration capture, alert decision, ACK, CSV row.
void onBodySample(unsigned long t) {
  if (calLeft > 0) {
    calPSum += body.pitch; calRSum += body.roll;
    if (--calLeft == 0) {
      pitchRef = calPSum / CAL_SAMPLES;
      rollRef  = calRSum / CAL_SAMPLES;
      calibrated = true;
      newCalId();
      if (gate.alert) setAlert(false, 0);
      gate.reset();
      flashLed();
      Line l;
      l.str("# calibrated upright: pitch=").fix(pitchRef, 1).str(" roll=").fix(rollRef, 1)
       .str(" calid=").u32(calId);
      out.event(l);
    }
  }

  float dPitch = body.pitch - pitchRef, dRoll = body.roll - rollRef;
  float dev = calibrated ? deviation(body.pitch, body.roll, pitchRef, rollRef) : 0;
  if (calibrated && calLeft == 0 && gate.update(dev, t)) setAlert(gate.alert, dev);

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

void pollRadio() {
  if (!radio.available()) return;              // also sends the ACK (+ payload)
  uint8_t buf[32];                             // = the DPL max payload
  uint8_t len = radio.getDynamicPayloadSize();
  if (len > sizeof(buf)) len = sizeof(buf);
  radio.read(buf, len);
  BodyPacket pkt;
  if (!parseBodyPacket(buf, len, pkt)) { badPkts++; return; }   // not a sample at all

  unsigned long t = millis();
  checkBodyLink(t);                            // a timed-out link is declared lost (alert
                                               // cleared, capture aborted) before this packet
  SeqKind k = seqs.onPacket(pkt.boot, pkt.seq, t);
  // Not a new sample: it doesn't keep the link up either, or a stream of them
  // would hold bodyUp (and a stale alert in the ACK) without fresh decisions.
  if (k == SEQ_DUP || k == SEQ_OLD) return;
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

void setup() {
  Serial.begin(115200);
  for (int b = 0; b < 4; b++) pinMode(SEL_PINS[b], OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(CAL_LED, OUTPUT);
  digitalWrite(CAL_LED, !LED_ON);
  analogReference(AR_VDD4);                    // full scale = VDD -> ratiometric
  analogReadResolution(12);
  analogSampleTime(10);                        // us; source impedance <= R_FIXED

  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // give the USB monitor a moment

  radioFailed = !radio.begin();
  radio.setChannel(RF_CHANNEL);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openReadingPipe(RF_PIPE, RF_ADDR);
  calId = randomId();                          // a chair reboot shouldn't reuse old ids
  updateAck(false);                            // preload before the first packet
  radio.startListening();                      // RX role

  out.poll();                                  // header goes out once a host is there
}

void report(unsigned long t) {
  Line l;
  l.str("# rx pkts=").u32(seqs.pkts).str(" lost=").u32(seqs.lost)
   .str(" dup=").u32(seqs.dup).str(" old=").u32(seqs.old).str(" rst=").u32(seqs.rst)
   .str(" bad=").u32(badPkts)
   .str(" sdrop=").u32(out.rowDrops()).str(" edrop=").u32(out.eventDrops())
   .str(" dt=").u32(t - lastReport);
  if (!out.stats(l)) return;                   // host not reading: keep counting
  lastReport = t;
  seqs.clearCounts();
  badPkts = 0;
  out.clearDrops();
}

void loop() {
  pollRadio();
  out.poll();
  pollSerial();
  pollButton();
  updateLed();

  unsigned long t = millis();
  checkBodyLink(t);
  if (t - lastScan >= SCAN_MS) {
    lastScan = t;
    scanFsrs();
    if (!bodyUp) printRow(t, 0, 0, 0);
  }

  if (t - lastReport >= 1000) report(t);
}
