// Integration test: the real firmware/chair/chair.ino on host stubs (stub/),
// fed with body packets through a fake radio, read through a fake USB serial.
#include "check.h"
#include "stub/Arduino.h"
#include "stub/nrf_to_nrf.h"
#include "stub/BleAdv.h"

const int SEATED = 2000;                       // every FSR: someone seated (an empty seat never alerts)

#include <vector>

namespace sim { uint32_t now = 1; int pin[32]; int adc = SEATED; }
FakeSerial Serial;
static FakeRng rng = {};
FakeRng *NRF_RNG = &rng;

#include "../chair/chair.ino"

static bool hostReading = true;

// Advance time 1 ms per loop() pass.
static void runFor(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    sim::now++;
    loop();
    if (hostReading) Serial.hostReads();
  }
}

static void pushPkt(const BodyPacket &p) {
  const uint8_t *b = (const uint8_t *)&p;
  radio.rx.emplace_back(b, b + sizeof(p));
}

// A body sample; its report: none yet (no ChairAck held).
static void push(uint8_t boot, uint16_t seq, float pitch, float roll = 0) {
  BodyPacket p = {};
  p.ver = PROTO_VER; p.age = AGE_NONE;
  p.seq = seq; p.boot = boot; p.pitch = pitch; p.roll = roll;
  pushPkt(p);
}

// One body sample, then the rest of its 50 ms period.
static void sample(uint8_t boot, uint16_t seq, float pitch, float roll = 0) {
  push(boot, seq, pitch, roll); runFor(50);
}

static ChairAck sentAck() {
  ChairAck a = {};
  memcpy(&a, radio.ackPayload.data(), sizeof(a));
  return a;
}

static std::vector<std::string> split(const std::string &l) {
  std::vector<std::string> f;
  size_t p = 0, e;
  while ((e = l.find(',', p)) != std::string::npos) { f.push_back(l.substr(p, e - p)); p = e + 1; }
  f.push_back(l.substr(p));
  return f;
}

static std::vector<std::string> outLines() {
  std::vector<std::string> v;
  size_t p = 0, e;
  while ((e = Serial.out.find("\r\n", p)) != std::string::npos) { v.push_back(Serial.out.substr(p, e - p)); p = e + 2; }
  CHECK_EQ(p, Serial.out.size());              // only whole lines
  return v;
}

static int count(const std::string &prefix) {
  int n = 0;
  for (auto &l : outLines()) n += l.compare(0, prefix.size(), prefix) == 0;
  return n;
}

// Sum of key=N over every "# rx" stats line printed so far.
static long rxSum(const std::string &key) {
  long n = 0;
  for (auto &l : outLines()) {
    if (l.compare(0, 4, "# rx")) continue;
    size_t p = l.find(" " + key + "=");
    if (p != std::string::npos) n += atol(l.c_str() + p + key.size() + 2);
  }
  return n;
}

static long lastRx(const std::string &key) {
  long n = -1;
  for (auto &l : outLines()) {
    if (l.compare(0, 4, "# rx")) continue;
    size_t p = l.find(" " + key + "=");
    if (p != std::string::npos) n = atol(l.c_str() + p + key.size() + 2);
  }
  return n;
}

static uint16_t s = 0;                         // body seq

// Linked, calibrated upright at pitch 0, then alerting (pitch 20 for > 3 s).
static void calibrateAndAlert(uint8_t boot) {
  for (int i = 0; i < 5; i++) sample(boot, s++, 0);
  Serial.in = "c";
  for (int i = 0; i < CAL_SAMPLES + 2; i++) sample(boot, s++, 0);
  CHECK(calibrated);
  for (int i = 0; i < 80; i++) sample(boot, s++, 20);
  CHECK(gate.alert);
  CHECK(sentAck().flags & CF_ALERT);
}

static void testHeaderAndRows() {
  auto v = outLines();
  CHECK(v.size() >= 3);
  CHECK_STR(v[0].substr(0, 22), "# chair unit (brain): ");
  CHECK_STR(v[1], "# unit=raw");
  CHECK_STR(v[2], "t_ms,cal,calid,dev,alert,dpitch,droll,pitch,roll,boot,seq,"
                  "bctl,bcal,bcalid,balert,bmotor,btest,bseq,bage,imu,binh,vbat,c8,c9,c10,c11,c12,c13,c14,c15,"
                  "reason,mute,bmute");
  // a body-silent row: boot..bage, imu, binh, vbat all -1 ("no sample"), no decision; 33 fields
  auto f = split(v.back());
  CHECK_EQ(f.size(), 33u);
  CHECK_STR(f[30], "-1");
  CHECK_STR(f[31], "0");                       // not muted
  CHECK_STR(f[32], "-1");                      // body silent: its mute unknown
  CHECK_STR(f[2], std::to_string(calId));      // chair's reference id, even with no body
  for (int i = 9; i <= 21; i++) CHECK_STR(f[i], "-1");
}

// FSR units stay raw; the removed 'u'/'U' commands cannot inject headers.
static void testFixedRawUnits() {
  const int values[] = {0, 7, 8, 2048, 4095};
  const char *commands[] = {"", "u", "uu", "U", "u"};
  for (int n = 0; n < 5; n++) {
    sim::adc = values[n];
    Serial.in = commands[n];
    runFor(100);
    auto f = split(outLines().back());
    CHECK_EQ(f.size(), 33u);
    for (int i = 22; i < 30; i++) CHECK_STR(f[i], std::to_string(values[n]));
  }
  CHECK_EQ(count("# unit=ohms"), 0);
  CHECK_EQ(count("# unit=raw"), 1);
  Serial.dtrOn = false;
  runFor(100);
  Serial.dtrOn = true;
  runFor(100);
  CHECK_EQ(count("# unit=raw"), 2);           // reconnect automatically sends the header
  CHECK_EQ(count("t_ms,"), 2);
  sim::adc = SEATED;
}

// Only duplicates / stale packets for > 1 s: no new sample, so the link must
// time out, the alert be cleared (and the ACK say so), a capture aborted.
static void testDupsDontKeepLinkUp() {
  const uint8_t B = 0x11;
  calibrateAndAlert(B);
  uint16_t last = s - 1;
  float pitchRef0 = pitchRef;
  int lost0 = count("# body link lost");
  for (int i = 0; i < 17; i++) { push(B, last, 20); runFor(50); }    // 850 ms of dups
  CHECK(bodyUp);                               // not yet: < LINK_TIMEOUT_MS since the last sample
  for (int i = 0; i < 10; i++) { push(B, last, 20); runFor(50); }    // > 1 s
  CHECK(!bodyUp);
  CHECK(!gate.alert);
  CHECK(!(sentAck().flags & CF_ALERT));
  CHECK(sentAck().flags & CF_CAL);             // calibration itself survives
  CHECK_EQ(count("# body link lost"), lost0 + 1);
  runFor(1000);
  CHECK(rxSum("dup") >= 27);

  // a new sample brings it back (same session, after an outage)
  int up0 = count("# body link up boot=11 resumed");
  sample(B, s++, 0);
  CHECK(bodyUp);
  CHECK_EQ(count("# body link up boot=11 resumed"), up0 + 1);

  // a capture interrupted by > 1 s of stale (OLD) packets is discarded.
  // (Past the timeout, a seq going back reads as a body restart — the boot-id
  // collision heuristic — but only after the link was declared lost.)
  for (int i = 0; i < 3; i++) sample(B, s++, 0);
  Serial.in = "c";
  for (int i = 0; i < 5; i++) sample(B, s++, 30);
  CHECK(calLeft > 0);
  uint16_t stale = s - 3;
  lost0 = count("# body link lost");
  for (int i = 0; i < 25; i++) { push(B, stale, 30); runFor(50); }
  CHECK_EQ(count("# body link lost"), lost0 + 1);
  CHECK_EQ(calLeft, 0);
  CHECK(pitchRef == pitchRef0);                // old reference kept, capture not applied
  runFor(1000);
  CHECK(rxSum("old") >= 19);
  for (int i = 0; i < 25; i++) sample(B, s++, 0);
  CHECK(bodyUp);
  CHECK_EQ(calLeft, 0);
  CHECK(pitchRef == pitchRef0);
}

static void testRebootWrapAndStats() {
  const uint8_t B = 0x11;
  runFor(2000);                                // outage, then the same session near the wrap
  s = 1000;
  sample(B, s++, 0);                           // resumed (seq forward)
  for (s = 65533; s != 65535; s++) sample(B, s, 0);   // (unexplained jump: a restart)
  runFor(1100);
  long lost0 = rxSum("lost"), rst0 = rxSum("rst");
  // seq wrap is an ordinary step
  for (int i = 0; i < 4; i++) sample(B, s++, 0);
  CHECK_EQ(s, 3);
  // a real gap while linked: 2 missing
  s += 2;
  for (int i = 0; i < 3; i++) sample(B, s++, 0);
  runFor(1000);
  CHECK_EQ(rxSum("lost") - lost0, 2);
  // body reboots (outage, new boot id, seq back to 0): no lost, one restart
  s = 1000;
  for (int i = 0; i < 3; i++) sample(B, s++, 0);
  runFor(2000);
  CHECK(!bodyUp);
  int up0 = count("# body link up boot=22 new");
  for (s = 0; s < 3; s++) sample(0x22, s, 0);
  CHECK_EQ(count("# body link up boot=22 new"), up0 + 1);
  // fast reboot, no outage: "new session" without lost/up
  int ns0 = count("# body new session boot=33");
  for (s = 0; s < 3; s++) sample(0x33, s, 0);
  CHECK(bodyUp);
  CHECK_EQ(count("# body new session boot=33"), ns0 + 1);
  runFor(1000);
  CHECK_EQ(rxSum("lost") - lost0, 2);
  CHECK_EQ(rxSum("rst") - rst0, 2);
  CHECK_EQ(lastRx("sdrop"), 0);
}

// Host holds DTR but stops reading: the chair keeps serving the radio and
// deciding; output is dropped (counted), nothing blocks, no partial line.
static void testStalledHost() {
  const uint8_t B = 0x33;
  hostReading = false;
  size_t out0 = Serial.out.size();
  int ackWrites0 = radio.ackWrites;
  bool sawAlert = false;
  for (int i = 0; i < 100; i++) { sample(B, s++, 25); sawAlert |= gate.alert; }   // 5 s slouched
  CHECK(radio.rx.empty());                     // every packet read
  CHECK(radio.ackWrites - ackWrites0 >= 100);  // a fresh ACK payload per sample
  CHECK(sawAlert);
  CHECK(sentAck().flags & CF_ALERT);
  CHECK_EQ(Serial.blocked, 0);
  CHECK(Serial.out.size() - out0 <= Serial.cap);
  hostReading = true;
  Serial.hostReads();
  runFor(5);                                   // the held-back stats line goes out first
  CHECK(lastRx("sdrop") > 50);                 // rows lost while stalled
  CHECK(lastRx("dt") > 1000);                  // stats line held back, then the real span
  outLines();                                  // whole lines only
}

static std::vector<std::string> lastRowFields() {
  auto v = outLines();
  for (size_t i = v.size(); i--;) if (!v[i].empty() && v[i][0] != '#') return split(v[i]);
  return {};
}
// chair CSV columns
enum { C_CAL = 1, C_CALID, C_DEV, C_ALERT, C_BOOT = 9, C_SEQ, C_BCTL, C_BCAL, C_BCALID,
       C_BALERT, C_BMOTOR, C_BTEST, C_BSEQ, C_BAGE, C_IMU, C_BINH, C_FSR,
       C_REASON = 30, C_MUTE, C_BMUTE };

// The ACK preloaded after each sample carries the decision of the row just
// printed, with that row's boot/seq as its basis; other rewrites say so.
static void testAckBasisAndCalId() {
  const uint8_t B = 0x44;
  runFor(2000);                                // whatever was linked times out
  CHECK(!bodyUp);
  CHECK_EQ(sentAck().basisBoot, 0);            // link timeout: not made on a sample
  CHECK_EQ(sentAck().basisSeq, 0);
  CHECK_EQ(sentAck().ver, PROTO_VER);
  for (s = 500; s < 505; s++) {
    sample(B, s, 0);
    auto f = lastRowFields();
    ChairAck a = sentAck();
    CHECK_EQ(a.basisBoot, B);
    CHECK_EQ(a.basisSeq, s);
    CHECK_STR(f[C_SEQ], std::to_string(s));
    CHECK_STR(f[C_BOOT], std::to_string(B));
    CHECK_STR(f[C_CAL], std::to_string(a.flags & CF_CAL ? 1 : 0));
    CHECK_STR(f[C_CALID], std::to_string(a.calId));
    CHECK_STR(f[C_ALERT], std::to_string(a.flags & CF_ALERT ? 1 : 0));
  }
  // a capture gets a new calId, announced in the event and the ACK
  uint8_t id0 = calId;
  Serial.in = "c";
  for (int i = 0; i < CAL_SAMPLES + 1; i++) sample(B, s++, 0);
  CHECK(calibrated);
  CHECK(calId != id0 && calId != 0);
  CHECK_EQ(sentAck().calId, calId);
  CHECK_EQ(count("# calibrated upright: pitch=0.0 roll=0.0 calid=" + std::to_string(calId) +
                 " fsr=2000.0,2000.0,2000.0,2000.0,2000.0,2000.0,2000.0,2000.0"), 1);
  // a motor test: same decision and basis, only buzz moves
  ChairAck before = sentAck();
  Serial.in = "m";
  runFor(1);
  CHECK_EQ(sentAck().buzz, (uint8_t)(before.buzz + 1));
  CHECK_EQ(sentAck().basisSeq, before.basisSeq);
  CHECK_EQ(sentAck().basisBoot, B);
  // clearing: new calId, no basis until the next sample
  uint8_t id1 = calId;
  Serial.in = "r";
  runFor(1);
  CHECK(calId != id1 && calId != 0);
  CHECK_EQ(sentAck().calId, calId);
  CHECK_EQ(sentAck().basisBoot, 0);
  CHECK(!(sentAck().flags & CF_CAL));
  CHECK_EQ(count("# calibration cleared calid=" + std::to_string(calId)), 1);
  sample(B, s++, 0);
  CHECK_EQ(sentAck().basisBoot, B);
  // calId wraps 255 -> 1, never 0
  calId = 255;
  Serial.in = "r";
  runFor(1);
  CHECK_EQ(calId, 1);
}

// What the body reports in its packet lands in the b* columns verbatim.
static void testBodyReportColumns() {
  const uint8_t B = 0x44;
  BodyPacket p = {};
  p.ver = PROTO_VER; p.boot = B; p.seq = s++; p.pitch = 1; p.roll = 2;
  p.ctl = CTL_CHAIR; p.flags = BF_CAL | BF_ALERT | BF_MOTOR; p.calId = 77;
  p.age = 2; p.ackSeq = (uint16_t)(p.seq - 2);
  pushPkt(p);
  runFor(50);
  auto f = lastRowFields();
  CHECK_STR(f[C_BCTL], "1");
  CHECK_STR(f[C_BCAL], "1");
  CHECK_STR(f[C_BCALID], "77");
  CHECK_STR(f[C_BALERT], "1");
  CHECK_STR(f[C_BMOTOR], "1");
  CHECK_STR(f[C_BTEST], "0");
  CHECK_STR(f[C_BSEQ], std::to_string((uint16_t)(p.seq - 2)));
  CHECK_STR(f[C_BAGE], "2");
  // fallback, a manual test buzz, no alert; age unknown -> bseq/bage -1
  p.seq = s++; p.ctl = CTL_FALLBACK; p.flags = BF_CAL | BF_MOTOR | BF_TEST; p.age = AGE_NONE; p.ackSeq = 0;
  pushPkt(p);
  runFor(50);
  f = lastRowFields();
  CHECK_STR(f[C_BCTL], "2");
  CHECK_STR(f[C_BALERT], "0");
  CHECK_STR(f[C_BMOTOR], "1");
  CHECK_STR(f[C_BTEST], "1");
  CHECK_STR(f[C_BSEQ], "-1");
  CHECK_STR(f[C_BAGE], "-1");
}

// 'q'/'v' set CF_MUTE in the ACK (decision and basis untouched); the row
// shows the chair's mute and the body's report; a host closing the port
// unmutes (the safe default), and a reconnect doesn't bring it back.
static void testMotorMute() {
  const uint8_t B = 0x45;
  for (int i = 0; i < 3; i++) sample(B, s++, 0);
  CHECK(!(sentAck().flags & CF_MUTE));
  ChairAck before = sentAck();
  int on0 = count("# motor muted"), off0 = count("# motor unmuted");
  Serial.in = "q";
  runFor(1);
  CHECK(muted);
  CHECK_EQ(sentAck().flags, before.flags | CF_MUTE);
  CHECK_EQ(sentAck().basisSeq, before.basisSeq);
  CHECK_EQ(sentAck().basisBoot, before.basisBoot);
  CHECK_EQ(count("# motor muted (alerts still decided and logged), from serial"), on0 + 1);
  sample(B, s++, 0);
  CHECK(sentAck().flags & CF_MUTE);            // every new decision keeps it
  auto f = lastRowFields();
  CHECK_STR(f[C_MUTE], "1");
  CHECK_STR(f[C_BMUTE], "0");                  // the body hasn't applied it yet
  BodyPacket p = {};
  p.ver = PROTO_VER; p.boot = B; p.seq = s++; p.pitch = 0; p.roll = 0;
  p.ctl = CTL_CHAIR; p.flags = BF_MUTE; p.calId = calId; p.age = AGE_NONE;
  pushPkt(p);
  runFor(50);
  CHECK_STR(lastRowFields()[C_BMUTE], "1");
  Serial.in = "Q";                             // re-sent: stays muted, logged again
  runFor(1);
  CHECK(muted);
  CHECK_EQ(count("# motor muted"), on0 + 2);
  Serial.in = "v";
  runFor(1);
  CHECK(!muted);
  CHECK(!(sentAck().flags & CF_MUTE));
  CHECK_EQ(count("# motor unmuted, from serial"), off0 + 1);
  // the capture tool closes the port: unmuted, and still so after it returns
  Serial.in = "q";
  runFor(1);
  CHECK(sentAck().flags & CF_MUTE);
  Serial.dtrOn = false;
  runFor(5);
  CHECK(!muted);
  CHECK(!(sentAck().flags & CF_MUTE));
  Serial.dtrOn = true;
  runFor(5);
  CHECK(!muted);
  sample(B, s++, 0);
  CHECK_STR(lastRowFields()[C_MUTE], "0");
  CHECK(!(sentAck().flags & CF_MUTE));
}

// Packets of another format / incoherent ones are counted as bad and are no
// sample: no row, no link renewal, no decision.
static void testRejectedPackets() {
  const uint8_t B = 0x44;
  runFor(1000);
  long bad0 = rxSum("bad"), ver0 = rxSum("ver");
  CHECK(!bodyUp);
  // a v1 body (12 bytes: seq, flags, boot, pitch, roll) is never accepted
  for (int i = 0; i < 30; i++) {
    uint8_t v1[12] = {(uint8_t)i, 0, 0, 0x5a};
    radio.rx.emplace_back(v1, v1 + sizeof v1);
    runFor(50);
  }
  CHECK(!bodyUp);
  CHECK_EQ(count("# body link up boot=5a"), 0);
  // wrong version / incoherent v3 while linked: the link still times out
  sample(B, s++, 0);
  CHECK(bodyUp);
  int lost0 = count("# body link lost");
  for (int i = 0; i < 25; i++) {
    BodyPacket p = {};
    p.ver = PROTO_VER; p.boot = B; p.seq = s++; p.age = AGE_NONE;
    if (i % 3 == 0) p.ver = 1;
    else if (i % 3 == 1) p.pitch = NAN;
    else p.flags = BF_ALERT;                   // alert without a reference
    pushPkt(p);
    runFor(50);
  }
  CHECK(!bodyUp);
  CHECK_EQ(count("# body link lost"), lost0 + 1);
  runFor(1000);
  CHECK_EQ(rxSum("bad") - bad0, 16);
  CHECK_EQ(rxSum("ver") - ver0, 39);
}

// Widest possible row, at t_ms = UINT32_MAX: every field at its
// longest, it must still go out whole through the real 256-byte USB FIFO.
static void testWidestRow() {
  const uint8_t B = 255;
  runFor(1100);
  for (int i = 0; i < 3; i++) { push(B, s++, 90, 180); runFor(50); }
  Serial.in = "c";
  for (int i = 0; i < CAL_SAMPLES + 1; i++) { push(B, s++, 90, 180); runFor(50); }
  CHECK(calibrated);
  calId = 255;
  lastScan = 0xFFFFFFF0u;                      // no scan row in between
  lastReport = 0xFFFFFFF0u;
  gate.alert = true;
  BodyPacket p = {};
  p.ver = PROTO_VER; p.boot = B; p.seq = 65535; p.pitch = -90; p.roll = -180;
  p.ctl = CTL_FALLBACK; p.flags = BF_CAL | BF_ALERT | BF_MOTOR | BF_TEST | BF_MUTE; p.calId = 255;
  p.age = 254; p.ackSeq = 10000; p.vbat = 65535;
  for (int i = 0; i < 8; i++) fsr[i] = 4095;
  sim::adc = 4095;
  muted = true;
  sim::now = 0xFFFFFFFEu;
  seqs = SeqTracker();                         // a fresh body session (seq 65535 not a dup)
  bodyUp = true;
  lastBodyAt = sim::now - 50;
  pushPkt(p);
  Serial.hostReads();
  uint32_t d0 = out.rowDrops();
  sim::now++;
  loop();
  auto v = outLines();
  std::string row = v.back();
  printf("  widest row (%zu bytes): %s\n", row.size(), row.c_str());
  CHECK_EQ(out.rowDrops(), d0);
  CHECK(row.size() <= Line::CAP);
  std::string fsrs;
  for (int i = 0; i < 8; i++) fsrs += ",4095";
  CHECK_STR(row, "4294967295,1,255,402.5,1,-180.0,-360.0,-90.0,-180.0,255,65535,"
                 "2,1,255,1,1,1,10000,254,0,0,65535" + fsrs + ",1,1,1");
  CHECK_EQ(row.size(), 141u);                  // reason -1 / bmute -1: 1 byte wider each
  seqs.clearCounts();
  s = 0;
  muted = false;
  runFor(1);
  sim::adc = SEATED;

  // stats line with every counter at its maximum (sdrop/edrop are 0 here:
  // 9 digits each of slack must remain for them)
  runFor(5);
  seqs.pkts = seqs.lost = seqs.dup = seqs.old = seqs.rst = badPkts = versionPkts = 0xFFFFFFFFu;
  Serial.hostReads();
  lastReport = sim::now + 2;                   // dt = UINT32_MAX
  sim::now++;
  report(sim::now);
  v = outLines();
  printf("  widest stats (%zu bytes + 18 for sdrop/edrop): %s\n", v.back().size(), v.back().c_str());
  CHECK_STR(v.back().substr(0, 16), "# rx pkts=429496");
  CHECK(v.back().size() + 18 <= Line::CAP);
  // and every header line went out (a too-long one would be skipped silently)
  int hdr = 0;
  for (auto &l : v) hdr += l.compare(0, 23, "# chair unit (brain): F") == 0;
  CHECK(hdr >= 1);
  for (uint8_t i = 0; i < 8; i++) { Line l; if (chairHeader(i, l)) CHECK(l.ok()); }
}

// Per-channel FSR load: the channel the mux selects (S0..S3 = D10, D3, D2, D1).
static int chAdc[16];
static int muxAdc() {
  int ch = 0;
  for (int b = 0; b < 4; b++) ch |= (sim::pin[SEL_PINS[b]] & 1) << b;
  return chAdc[ch];
}
static void load(int seatFL, int seatFR, int seatRL, int seatRR, int back = SEATED) {
  for (int c = 8; c < 16; c++) chAdc[c] = back;
  chAdc[8] = seatFL; chAdc[9] = seatFR; chAdc[10] = seatRL; chAdc[11] = seatRR;   // FSR_SPOT_CH
}

// The fusion rule in the sketch: FSR baseline captured with the reference,
// reason column, slumped / reclined / leaning / empty seat, spot map sides.
static const char *fusionDump = nullptr;      // argv[1]: write this test's chair output

static void testFusion() {
  const uint8_t B = 0x55;
  size_t mark = Serial.out.size();
  sim::adcFn() = muxAdc;
  Serial.in = "r";
  runFor(1100);
  CHECK(!calibrated);
  // perched on the front edge (seat rear unloaded): the capture is rejected
  load(2500, 2500, 0, 0);
  for (int i = 0; i < 3; i++) sample(B, s++, 0);
  uint8_t id0 = calId;
  Serial.in = "c";
  for (int i = 0; i < CAL_SAMPLES + 1; i++) sample(B, s++, 0);
  CHECK(!calibrated);
  CHECK_EQ(calId, id0);
  CHECK(!(sentAck().flags & CF_CAL));
  CHECK_EQ(count("# calibrate rejected: seat rear unloaded (sit back fully and recalibrate) rear=0"), 1);
  CHECK(rejectShown);                          // LED: red fast for REJECT_LED_MS
  CHECK_EQ(sim::pin[LED_GREEN], 255);
  CHECK_STR(lastRowFields()[C_REASON], "-1");
  for (int i = 0; i < 50; i++) sample(B, s++, 0, 3);   // slid-like, small IMU: no alert, uncalibrated
  CHECK(!gate.alert);
  CHECK(!rejectShown);
  // sitting back fully: accepted
  load(2000, 2200, 2600, 2400);
  for (int i = 0; i < 3; i++) sample(B, s++, 0);
  Serial.in = "c";
  for (int i = 0; i < CAL_SAMPLES + 1; i++) sample(B, s++, 0);
  CHECK(calibrated);
  CHECK_EQ(count("# calibrated upright: pitch=0.0 roll=0.0 calid=" + std::to_string(calId) +
                 " fsr=2000.0,2200.0,2600.0,2400.0,2000.0,2000.0,2000.0,2000.0"), 1);
  CHECK(fusionBase.rearOk && fusionBase.lrOk);
  CHECK(fabsf(fusionBase.rear - 5000) < 0.01f);
  auto reason = [] { return lastRowFields()[C_REASON]; };   // last column
  auto run = [](float pitch, float roll, int n) {
    for (int i = 0; i < n; i++) { push(B, s++, pitch, roll); runFor(50); }
  };
  run(0, 0, 5);
  CHECK_STR(reason(), "0");
  // slid forward: seat rear unloaded, IMU barely moves -> alert after the hold
  load(2600, 2800, 0, 0);
  run(0, 4, 50);
  CHECK_STR(reason(), "2");
  CHECK(!gate.alert);
  run(0, 4, 15);
  CHECK(gate.alert);
  CHECK(sentAck().flags & CF_ALERT);
  load(2000, 2200, 2600, 2400);
  run(0, 0, 2);
  CHECK(!gate.alert);
  CHECK_STR(reason(), "0");
  // reclined: hips back, backward tilt accepted
  run(0, 20, 100);
  CHECK_STR(reason(), "4");
  CHECK(!gate.alert);
  // same tilt with the hips half forward: IMU deviation
  load(2400, 2600, 1500, 1300);
  run(0, 20, 70);
  CHECK_STR(reason(), "1");
  CHECK(gate.alert);
  load(2000, 2200, 2600, 2400);
  run(0, 0, 2);
  // small lean left (dpitch < 0) with the seat's left side loaded (C8, C10)
  load(2800, 1800, 3400, 1800);
  run(-6, 0, 70);
  CHECK_STR(reason(), "3");
  CHECK(gate.alert);
  load(1400, 2800, 1800, 3400);                // the seat moved right instead: not a lean
  run(-6, 0, 2);
  CHECK_STR(reason(), "0");
  CHECK(!gate.alert);
  // nobody seated (only the backrest touched): never alerts, even far off
  load(0, 0, 0, 0, 1500);
  run(30, 0, 100);
  CHECK_STR(reason(), "5");
  CHECK(!gate.alert);
  // a rejected recapture keeps the reference in use (as an aborted one)
  uint8_t id1 = calId;
  load(2600, 2600, 300, 200);
  for (int i = 0; i < 3; i++) sample(B, s++, 0);
  Serial.in = "c";
  for (int i = 0; i < CAL_SAMPLES + 1; i++) sample(B, s++, 0);
  CHECK(calibrated);
  CHECK_EQ(calId, id1);
  CHECK_EQ(count("# calibrate rejected: seat rear unloaded (sit back fully and recalibrate) rear=500"), 1);
  CHECK(fusionBase.rearOk);
  // cleared: no decision, baseline gone
  Serial.in = "r";
  runFor(1);
  run(0, 0, 2);
  CHECK_STR(reason(), "-1");
  CHECK(!fusionBase.rearOk);
  sim::adcFn() = nullptr;
  if (fusionDump) {                            // replayed by analysis/test_fusion.py
    size_t h = Serial.out.rfind("t_ms,", mark);
    FILE *fp = fopen(fusionDump, "w");
    CHECK(fp != nullptr && h != std::string::npos);
    if (!fp) return;
    std::string hdr = Serial.out.substr(h, Serial.out.find("\r\n", h) - h);
    fprintf(fp, "# unit=raw\n%s\n%s", hdr.c_str(), Serial.out.substr(mark).c_str());
    fclose(fp);
  }
}

// The BLE beacon: rate, never with an ESB packet pending, contents (Beacon.h
// layout), and the calibration cleared on an empty seat only without a host.
static uint16_t le16(const std::vector<uint8_t> &p, int i) { return p[i] | p[i + 1] << 8; }
static uint16_t fsrOf(const std::vector<uint8_t> &p, int ch) {   // ch 0..7 = c8..c15
  const uint8_t *b = p.data() + 12 + 15 + 3 * (ch / 2);
  uint32_t v = b[0] | b[1] << 8 | (uint32_t)b[2] << 16;
  return (ch & 1 ? v >> 12 : v) & 0xFFF;
}

static void testBeacon() {
  const uint8_t B = 0x66;
  sim::adcFn() = muxAdc;
  load(2000, 2000, 2000, 2000);
  auto &sent = sim::bleSent();
  // body silent: on the timer, ~20 Hz
  runFor(1100);                                // link times out first
  CHECK(!bodyUp);
  size_t n0 = sent.size();
  runFor(1000);
  CHECK(sent.size() - n0 >= 19 && sent.size() - n0 <= 21);
  CHECK(!(sent.back()[12 + 4] & BK_BODY));
  CHECK_EQ(le16(sent.back(), 12 + 13), 0xFFFF);
  CHECK_EQ((int8_t)sent.back()[12 + 8], -1);

  // linked: right after every sample, ~20 Hz
  load(2100, 2200, 2300, 2400, 1234);
  for (int i = 0; i < 3; i++) sample(B, s++, 0);
  Serial.in = "c";
  for (int i = 0; i < CAL_SAMPLES + 2; i++) sample(B, s++, 0);
  CHECK(calibrated);
  n0 = sent.size();
  for (int i = 0; i < 40; i++) {               // 2 s
    BodyPacket p = {};
    p.ver = PROTO_VER; p.age = AGE_NONE; p.seq = s++; p.boot = B; p.pitch = 4.25f; p.roll = -2.5f;
    p.ctl = CTL_CHAIR; p.flags = BF_CAL | BF_MUTE; p.calId = calId; p.vbat = 3987;
    pushPkt(p); runFor(50);
  }
  CHECK_EQ(sent.size() - n0, (size_t)40);
  CHECK_EQ(sim::blePendingSends(), 0);
  const std::vector<uint8_t> &p = sent.back();
  CHECK_EQ(p.size(), (size_t)ADV_PDU_LEN);
  CHECK_EQ(p[0], 0x42);
  CHECK_EQ(p[1], 37);
  CHECK_EQ(p[7], 0xC6);                        // AdvA (stub address), top bits = static random
  CHECK_EQ(p[8], 30);
  CHECK_EQ(p[9], 0xFF);
  CHECK_EQ(le16(p, 10), BEACON_COMPANY);
  CHECK_EQ(p[12], BEACON_VER);
  CHECK_EQ((uint8_t)(p[13] - sent[sent.size() - 2][13]), 1);
  CHECK_EQ(le16(p, 14), (uint16_t)lastBeacon);
  CHECK_EQ(p[16], BK_CAL | BK_BODY | BK_USB);
  CHECK_EQ(p[17], CTL_CHAIR | BB_CAL | BB_MUTE);
  CHECK_EQ(p[18], calId);
  CHECK_EQ((int8_t)p[19], R_OK);
  CHECK_EQ((int8_t)p[20], IMU_OK);
  CHECK_EQ((int16_t)le16(p, 21), 43);          // 4.25 deg -> 0.1 deg, rounded
  CHECK_EQ((int16_t)le16(p, 23), -25);
  CHECK_EQ(le16(p, 25), 3987);
  const int want[8] = {2100, 2200, 2300, 2400, 1234, 1234, 1234, 1234};
  for (int c = 0; c < 8; c++) CHECK_EQ(fsrOf(p, c), want[c]);

  // empty seat with a host on USB: kept
  load(0, 0, 0, 0, 0);
  for (int i = 0; i < 240; i++) sample(B, s++, 0);   // 12 s
  CHECK(calibrated);
  // the host goes away: the seat has been empty long enough already
  Serial.dtrOn = false;
  sample(B, s++, 0);
  CHECK(!calibrated);
  for (int i = 0; i < 5; i++) sample(B, s++, 0);
  CHECK(!(sent.back()[12 + 4] & (BK_CAL | BK_USB)));
  // no host, seated and calibrated (button), then empty: cleared after SEAT_EMPTY_CLEAR_MS
  load(2100, 2200, 2300, 2400);
  for (int i = 0; i < 3; i++) sample(B, s++, 0);
  startCalibration("button");
  for (int i = 0; i < CAL_SAMPLES + 2; i++) sample(B, s++, 0);
  CHECK(calibrated);
  load(2100, 2200, 0, 0, 0);                   // seat sum 4300: still seated
  for (int i = 0; i < 240; i++) sample(B, s++, 0);
  CHECK(calibrated);
  load(0, 0, 0, 0, 0);
  for (int i = 0; i < 190; i++) sample(B, s++, 0);   // 9.5 s
  CHECK(calibrated);
  for (int i = 0; i < 20; i++) sample(B, s++, 0);
  CHECK(!calibrated);
  Serial.dtrOn = true;
  runFor(100);
  load(2000, 2000, 2000, 2000);
  sim::adcFn() = nullptr;
}

//   ./test_chair [fusion.out]   (also dumps testFusion's chair output)
int main(int argc, char **argv) {
  if (argc > 1) fusionDump = argv[1];
  for (int &p : sim::pin) p = HIGH;            // button released, LEDs off
  rng.VALUE = 200;
  sim::bleRadio() = &radio;
  setup();
  CHECK_EQ(calId, 200);                        // random start (RNG)
  CHECK_EQ(sim::pin[VBAT_ENABLE], LOW);        // LiPo: VBAT sense safe, 100 mA charge
  CHECK_EQ(sim::pin[PIN_CHARGING_CURRENT], LOW);
  runFor(200);                                 // body silent: rows with seq -1
  CHECK_EQ(sim::pin[LED_RED], 0);              // orange, active LOW: duty = lit fraction inverted
  CHECK_EQ(sim::pin[LED_GREEN], 255 - RGB_ORANGE.g);
  CHECK_EQ(sim::pin[LED_BLUE], 255);
  testHeaderAndRows();
  testFixedRawUnits();
  testDupsDontKeepLinkUp();
  testRebootWrapAndStats();
  testStalledHost();
  testAckBasisAndCalId();
  testBodyReportColumns();
  testRejectedPackets();
  testMotorMute();
  testWidestRow();
  testFusion();
  testBeacon();
  CHECK_EQ(Serial.blocked, 0);
  return report("chair.ino");
}
