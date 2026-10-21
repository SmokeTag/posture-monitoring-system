// Integration test: the real firmware/chair/chair.ino on host stubs (stub/),
// fed with body packets through a fake radio, read through a fake USB serial.
#include "check.h"
#include "stub/Arduino.h"
#include "stub/nrf_to_nrf.h"

#include <vector>

namespace sim { uint32_t now = 1; int pin[32]; int adc = 0; }
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
static void sample(uint8_t boot, uint16_t seq, float pitch) { push(boot, seq, pitch); runFor(50); }

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
                  "bctl,bcal,bcalid,balert,bmotor,btest,bseq,bage,imu,binh,vbat,c8,c9,c10,c11,c12,c13,c14,c15");
  // a body-silent row: boot..bage, imu, binh, vbat all -1 ("no sample"), 30 fields
  auto f = split(v.back());
  CHECK_EQ(f.size(), 30u);
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
    CHECK_EQ(f.size(), 30u);
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
  sim::adc = 0;
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
       C_BALERT, C_BMOTOR, C_BTEST, C_BSEQ, C_BAGE, C_IMU, C_BINH, C_FSR };

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
  CHECK_EQ(count("# calibrated upright: pitch=0.0 roll=0.0 calid=" + std::to_string(calId)), 1);
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
  p.ctl = CTL_FALLBACK; p.flags = BF_CAL | BF_ALERT | BF_MOTOR | BF_TEST; p.calId = 255;
  p.age = 254; p.ackSeq = 10000; p.vbat = 65535;
  for (int i = 0; i < 8; i++) fsr[i] = 4095;
  sim::adc = 4095;
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
                 "2,1,255,1,1,1,10000,254,0,0,65535" + fsrs);
  CHECK_EQ(row.size(), 135u);
  seqs.clearCounts();
  s = 0;
  runFor(1);
  sim::adc = 0;

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

int main() {
  for (int &p : sim::pin) p = HIGH;            // button released, LEDs off
  rng.VALUE = 200;
  setup();
  CHECK_EQ(calId, 200);                        // random start (RNG)
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
  testWidestRow();
  CHECK_EQ(Serial.blocked, 0);
  return report("chair.ino");
}
