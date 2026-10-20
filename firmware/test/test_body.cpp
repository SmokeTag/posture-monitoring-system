// Integration test: the real firmware/body/body.ino on host stubs (stub/),
// with a fake radio deciding what each write() and its ACK payload look like.
// Starts 3 s before millis() wraps, so every scenario runs across the rollover.
#include "check.h"
#include "stub/Arduino.h"
#include "stub/nrf_to_nrf.h"

#include <vector>

namespace sim { uint32_t now = 0xFFFFFFFFu - 3000; int pin[32]; int adc = 0; float ax = 0, ay = 0, az = 1; }
FakeSerial Serial;
static FakeRng rng = {};
FakeRng *NRF_RNG = &rng;

#include "../body/body.ino"

static void runFor(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) { sim::now++; loop(); Serial.hostReads(); }
}

static void tilt(float deg) {
  sim::ax = -sinf(deg / RAD_TO_DEGF); sim::ay = 0; sim::az = cosf(deg / RAD_TO_DEGF);
}

static void ackWith(uint8_t flags, uint8_t buzz = 0) {
  ChairAck a = {};
  a.ver = PROTO_VER; a.calId = 9;
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
  a.ver = PROTO_VER; a.flags = CF_CAL; a.calId = 255; a.pitchRef = 90; a.rollRef = 180;
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
  ledState = true;
  sim::now++;
  loop();
  std::string row = lastRow();
  printf("  widest row (%zu bytes): %s\n", row.size(), row.c_str());
  CHECK_STR(row, "4294967295,65535,2,1,255,402.5,1,1,1,65235,254,-90.0,-180.0");
  CHECK(row.size() + 2 <= (size_t)Line::CAP + 2);
  CHECK_EQ(out.rowDrops(), 0u);
  // stats line with every counter at its maximum (+ 9 digits each of slack
  // for ack/sdrop/edrop, 0 here); header lines all fit
  chairLink.sent = chairLink.acked = chairLink.retx = chairLink.ctl = chairLink.bad = 0xFFFFFFFFu;
  chairLink.acked = 0;                         // noack = sent: 10 digits too
  lastReport = sim::now + 1;
  report(sim::now);
  auto v = outLines();
  printf("  widest stats (%zu bytes + 27 for ack/sdrop/edrop): %s\n", v.back().size(), v.back().c_str());
  CHECK_STR(v.back().substr(0, 16), "# tx sent=429496");
  CHECK(v.back().size() + 27 <= Line::CAP);
  imuFailed = radioFailed = true;
  for (uint8_t i = 0; i < 8; i++) { Line l; if (bodyHeader(i, l)) CHECK(l.ok()); }
}

int main() {
  rng.VALUE = 0x5a;
  for (int &p : sim::pin) p = HIGH;
  setup();
  runFor(100);
  auto v = outLines();
  CHECK(v.size() >= 2);
  CHECK(v[0].find("boot=5a") != std::string::npos);
  CHECK_STR(v[1], "t_ms,seq,ctl,cal,calid,dev,alert,motor,test,aseq,age,pitch,roll");
  CHECK(!radio.sent.empty());
  BodyPacket p0, chk;
  CHECK_EQ(radio.sent.back().size(), sizeof(BodyPacket));
  memcpy(&p0, radio.sent.back().data(), sizeof(p0));
  CHECK_EQ(p0.ver, PROTO_VER);
  CHECK_EQ(p0.boot, 0x5a);
  CHECK(parseBodyPacket(&p0, sizeof p0, chk));
  CHECK_EQ(p0.ctl, CTL_NONE);
  CHECK_EQ(p0.age, AGE_NONE);

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
  runFor(200);
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
    long sent, ack, noack, retx, ctl, bad;
    CHECK_EQ(sscanf(l.c_str(), "# tx sent=%ld ack=%ld noack=%ld retx=%ld ctl=%ld bad=%ld",
                    &sent, &ack, &noack, &retx, &ctl, &bad), 6);
    CHECK_EQ(ack, ctl + bad);
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
  // every packet sent in this run is a well-formed v2 packet
  int badSent = 0;
  for (auto &b : radio.sent) badSent += !parseBodyPacket(b.data(), (uint8_t)b.size(), chk);
  CHECK_EQ(badSent, 0);
  CHECK(sim::now < 0xFFFFFFFFu - 3000);        // the run crossed the millis() wrap
  CHECK_EQ(Serial.blocked, 0);
  outLines();
  testWidestRow();
  CHECK_EQ(Serial.blocked, 0);
  return report("body.ino");
}
