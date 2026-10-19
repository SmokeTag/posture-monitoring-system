// Integration test: the real firmware/chair/chair.ino on host stubs (stub/),
// fed with body packets through a fake radio, read through a fake USB serial.
#include "check.h"
#include "stub/Arduino.h"
#include "stub/nrf_to_nrf.h"

#include <vector>

namespace sim { uint32_t now = 1; int pin[32]; int adc = 0; }
FakeSerial Serial;
FakeRng *NRF_RNG = nullptr;

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

static void push(uint8_t boot, uint16_t seq, float pitch, float roll = 0) {
  BodyPacket p = {};
  p.seq = seq; p.boot = boot; p.pitch = pitch; p.roll = roll;
  const uint8_t *b = (const uint8_t *)&p;
  radio.rx.emplace_back(b, b + sizeof(p));
}

// One body sample, then the rest of its 50 ms period.
static void sample(uint8_t boot, uint16_t seq, float pitch) { push(boot, seq, pitch); runFor(50); }

static ChairAck sentAck() {
  ChairAck a = {};
  memcpy(&a, radio.ackPayload.data(), sizeof(a));
  return a;
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
  CHECK_STR(v[1], "# unit=ohms (-1 = open)");
  CHECK_STR(v[2], "t_ms,cal,dev,alert,led,dpitch,droll,pitch,roll,seq,c8,c9,c10,c11,c12,c13,c14,c15");
  // a body-silent row: seq -1, 18 fields
  std::string row = v.back();
  int commas = 0;
  for (char c : row) commas += c == ',';
  CHECK_EQ(commas, 17);
  CHECK(row.find(",-1,") != std::string::npos);
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

int main() {
  for (int &p : sim::pin) p = HIGH;            // button released, LEDs off
  setup();
  runFor(200);                                 // body silent: rows with seq -1
  testHeaderAndRows();
  testDupsDontKeepLinkUp();
  testRebootWrapAndStats();
  testStalledHost();
  return report("chair.ino");
}
