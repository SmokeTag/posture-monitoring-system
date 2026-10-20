// System test: the real firmware/chair/chair.ino AND firmware/body/body.ino on
// host stubs, linked by a fake ESB radio (the body's write() delivers to the
// chair and gets back the ACK payload the chair has preloaded at that moment).
// Each board has its own millis() (unrelated offsets): nothing may depend on
// comparing them. Checks the telemetry
// timeline of PostureLink.h (decision on N -> applied at N+1 -> reported in
// N+2), the join chair row seq == bseq, fallback in/out, outage recovery and
// a manual motor test.
//   ./test_system [chair.out body.out]   (also dumps both serial streams)
#include "check.h"
#include "stub/Arduino.h"
#include "stub/LSM6DS3.h"
#include "stub/Wire.h"
#include "stub/nrf_to_nrf.h"
#include "stub/Wire.h"
FakeWire Wire1;
#include <PostureLink.h>
#include <RandomId.h>
#include <SerialOut.h>

using namespace posture;

#include <map>
#include <vector>

namespace sim { uint32_t now = 0; int pin[32]; int adc = 0; float ax = 0, ay = 0, az = 1; }
FakeSerial Serial;                             // unused: each board has its own below
static FakeRng rng = {};
FakeRng *NRF_RNG = &rng;
FakeSerial chairSerial, bodySerial;
// Unrelated clocks; the body's wraps ~5 s in. (The chair's does not: its
// `unsigned long` timestamps are 64-bit on the host, 32-bit on the nRF52, so
// a host run across its wrap would not show what the board does.)
const uint32_t CHAIR_CLOCK = 0x80000000u + 12345, BODY_CLOCK = 0xFFFFFFFFu - 5003;

namespace chairfw {
#define Serial chairSerial
#define millis() (sim::now + CHAIR_CLOCK)
#include "../chair/chair.ino"
#undef millis
#undef Serial
}
namespace bodyfw {
#define Serial bodySerial
#define millis() (sim::now + BODY_CLOCK)
#include "../body/body.ino"
#undef millis
#undef Serial
}

static void runFor(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    sim::now++;
    bodyfw::loop();
    chairfw::loop();
    chairSerial.hostReads();
    bodySerial.hostReads();
  }
}

static void tilt(float deg) {
  sim::ax = -sinf(deg / bodyfw::RAD_TO_DEGF); sim::ay = 0; sim::az = cosf(deg / bodyfw::RAD_TO_DEGF);
}

// --- parse both CSV streams by their own header lines

typedef std::map<std::string, long> Row;

static std::vector<std::string> split(const std::string &l) {
  std::vector<std::string> f;
  size_t p = 0, e;
  while ((e = l.find(',', p)) != std::string::npos) { f.push_back(l.substr(p, e - p)); p = e + 1; }
  f.push_back(l.substr(p));
  return f;
}

static std::vector<Row> rows(const std::string &out) {
  std::vector<Row> v;
  std::vector<std::string> cols;
  size_t p = 0, e;
  while ((e = out.find("\r\n", p)) != std::string::npos) {
    std::string l = out.substr(p, e - p);
    p = e + 2;
    if (l.compare(0, 5, "t_ms,") == 0) { cols = split(l); continue; }
    if (l.empty() || l[0] == '#' || cols.empty()) continue;
    auto f = split(l);
    CHECK_EQ(f.size(), cols.size());
    Row r;
    for (size_t i = 0; i < f.size() && i < cols.size(); i++) r[cols[i]] = atol(f[i].c_str());
    v.push_back(r);
  }
  CHECK_EQ(p, out.size());                     // whole lines only
  return v;
}

// Last chair / body row (from `from` on) with seq == s.
static const Row *find(const std::vector<Row> &v, long s, size_t from = 0) {
  const Row *r = nullptr;
  for (size_t i = from; i < v.size(); i++) if (v[i].at("seq") == s) r = &v[i];
  return r;
}

// Every report that names a decision (bseq != -1) matches the chair row that
// made it (seq == bseq, same boot): cal, calid and — under chair control —
// alert. Returns how many reports were checked.
static int checkJoins(const std::vector<Row> &c) {
  int n = 0;
  for (size_t i = 0; i < c.size(); i++) {
    const Row &r = c[i];
    if (r.at("bseq") < 0) continue;
    CHECK_EQ(r.at("bage"), (r.at("seq") - r.at("bseq")) & 0xFFFF);
    const Row *d = nullptr;
    for (size_t j = i; j-- > 0 && c[j].at("boot") == r.at("boot");)
      if (c[j].at("seq") == r.at("bseq")) { d = &c[j]; break; }
    if (!d) continue;                          // decided before this capture
    n++;
    CHECK_EQ(r.at("bcal"), d->at("cal"));
    CHECK_EQ(r.at("bcalid"), d->at("calid"));
    if (r.at("bctl") == CTL_CHAIR) CHECK_EQ(r.at("balert"), d->at("alert") && !r.at("binh"));
  }
  return n;
}

int main(int argc, char **argv) {
  for (int &p : sim::pin) p = HIGH;
  rng.VALUE = 0x30; chairfw::setup();
  rng.VALUE = 0x5a; bodyfw::setup();
  chairSerial.hostReads(); bodySerial.hostReads();  // host drains startup headers
  bodyfw::radio.peer = &chairfw::radio;
  tilt(0);

  // link up: steady state body age 1 (aseq = seq-1), chair bage 2 (bseq = seq-2)
  runFor(2000);
  auto c = rows(chairSerial.out), b = rows(bodySerial.out);
  CHECK(c.size() > 30 && b.size() > 30);
  CHECK_EQ(c.back().at("boot"), 0x5a);
  CHECK_EQ(c.back().at("bctl"), CTL_CHAIR);
  CHECK_EQ(c.back().at("bage"), 2);
  CHECK_EQ(c.back().at("bseq"), (c.back().at("seq") - 2) & 0xFFFF);
  CHECK_EQ(c.back().at("calid"), 0x30);
  CHECK_EQ(c.back().at("bcalid"), 0x30);
  CHECK_EQ(b.back().at("ctl"), CTL_CHAIR);
  CHECK_EQ(b.back().at("age"), 1);
  CHECK_EQ(b.back().at("aseq"), (b.back().at("seq") - 1) & 0xFFFF);
  // the first packet holds nothing yet; its ACK is the chair's boot state:
  // valid control (link up) but made on no sample -> age unknown, then 1
  BodyPacket p0;
  memcpy(&p0, bodyfw::radio.sent.front().data(), sizeof p0);
  CHECK_EQ(p0.seq, 0);
  CHECK_EQ(p0.ctl, CTL_NONE);
  CHECK_EQ(p0.age, AGE_NONE);
  const Row *b0 = find(b, 0), *b1 = find(b, 1);
  CHECK(b0 && b1);
  if (b0 && b1) {
    CHECK_EQ(b0->at("ctl"), CTL_CHAIR);
    CHECK_EQ(b0->at("age"), -1);
    CHECK_EQ(b1->at("age"), 1);
  }

  // calibrate upright, then slouch: the chair's alert reaches the body one
  // sample later and is reported one more sample later
  chairSerial.in = "c";
  runFor(1500);
  CHECK(chairfw::calibrated);
  uint8_t cal1 = chairfw::calId;
  CHECK(cal1 != 0x30);
  size_t c0 = rows(chairSerial.out).size();
  tilt(20);
  runFor(4000);
  c = rows(chairSerial.out); b = rows(bodySerial.out);
  long N = -1;
  for (size_t i = c0; i < c.size(); i++) if (c[i].at("alert")) { N = c[i].at("seq"); break; }
  CHECK(N >= 0);
  printf("  chair alert decided on body sample N = %ld\n", N);
  const Row *cN = find(c, N), *cN1 = find(c, (N + 1) & 0xFFFF), *cN2 = find(c, (N + 2) & 0xFFFF);
  const Row *bN = find(b, N), *bN1 = find(b, (N + 1) & 0xFFFF);
  CHECK(cN && cN1 && cN2 && bN && bN1);
  if (cN && cN1 && cN2 && bN && bN1) {
    CHECK_EQ(cN->at("alert"), 1);
    CHECK_EQ(bN->at("alert"), 0);              // body row N: ACK of N carried basis N-1
    CHECK_EQ(bN->at("aseq"), (N - 1) & 0xFFFF);
    CHECK_EQ(bN1->at("alert"), 1);             // body row N+1: applies decision N
    CHECK_EQ(bN1->at("aseq"), N);
    CHECK_EQ(bN1->at("age"), 1);
    CHECK_EQ(bN1->at("calid"), cal1);
    CHECK_EQ(cN1->at("balert"), 0);            // packet N+1 left before that ACK
    CHECK_EQ(cN1->at("bseq"), (N - 1) & 0xFFFF);
    CHECK_EQ(cN2->at("balert"), 1);            // packet N+2 reports it
    CHECK_EQ(cN2->at("bseq"), N);
    CHECK_EQ(cN2->at("bage"), 2);
  }

  // manual motor test, upright, no alert: test + motor, not alert, on both sides
  tilt(0);
  runFor(1000);
  CHECK(!chairfw::gate.alert);
  CHECK(!bodyfw::alert);
  chairSerial.in = "m";
  runFor(150);
  c = rows(chairSerial.out); b = rows(bodySerial.out);
  CHECK_EQ(b.back().at("test"), 1);
  CHECK_EQ(b.back().at("motor"), 1);
  CHECK_EQ(b.back().at("alert"), 0);
  CHECK_EQ(c.back().at("btest"), 1);
  CHECK_EQ(c.back().at("bmotor"), 1);
  CHECK_EQ(c.back().at("balert"), 0);
  CHECK_EQ(c.back().at("alert"), 0);
  runFor(1000);

  // ACKs lost, packets still arriving: the body falls back (bctl 2) and keeps
  // reporting the reference it holds; the age of what it holds grows
  size_t f0 = rows(chairSerial.out).size();
  bodyfw::radio.downOk = false;
  tilt(20);
  runFor(5000);
  c = rows(chairSerial.out); b = rows(bodySerial.out);
  CHECK_EQ(b.back().at("ctl"), CTL_FALLBACK);
  CHECK_EQ(c.back().at("bctl"), CTL_FALLBACK);
  CHECK_EQ(c.back().at("bcalid"), c.back().at("calid"));   // same reference
  CHECK_EQ(c.back().at("balert"), 1);        // its own decision (the chair agrees here)
  CHECK(c.back().at("bage") > 90);
  CHECK_EQ(c.back().at("bage"), (c.back().at("seq") - c.back().at("bseq")) & 0xFFFF);
  long fb = 0;
  for (size_t i = f0; i < c.size(); i++) fb += c[i].at("bctl") == CTL_FALLBACK;
  CHECK(fb > 50);
  // ACKs back: chair control again, age 1 at once (the chair never lost it)
  bodyfw::radio.downOk = true;
  runFor(200);
  b = rows(bodySerial.out);
  CHECK_EQ(b.back().at("ctl"), CTL_CHAIR);
  CHECK_EQ(b.back().at("age"), 1);

  // full outage: the chair times out (its ACK loses its basis); the first ACK
  // after it carries no basis -> body age unknown for that sample, then 1
  tilt(0);
  runFor(500);
  bodyfw::radio.upOk = false;
  runFor(2500);
  CHECK(!chairfw::bodyUp);
  CHECK_EQ(chairfw::ack.basisBoot, 0);
  size_t r0 = rows(bodySerial.out).size();
  bodyfw::radio.upOk = true;
  runFor(500);
  b = rows(bodySerial.out);
  CHECK(r0 + 2 < b.size());
  if (r0 + 2 < b.size()) {
    size_t k = r0;
    while (k < b.size() && b[k].at("ctl") != CTL_CHAIR) k++;
    CHECK(k + 1 < b.size());
    if (k + 1 < b.size()) {
      CHECK_EQ(b[k].at("age"), -1);            // the chair's timed-out state, no basis
      CHECK_EQ(b[k].at("aseq"), -1);
      CHECK_EQ(b[k + 1].at("age"), 1);
    }
  }

  // A sensor failure interrupts a partial capture without losing the last
  // valid reference or the radio link. A new capture is refused while invalid.
  uint8_t savedCal = chairfw::calId;
  chairSerial.in = "c"; runFor(250);
  CHECK(chairfw::calLeft > 0);
  Wire1.forceStale = true; runFor(1100);
  CHECK(chairfw::bodyUp);
  CHECK_EQ(chairfw::body.imu, IMU_STALE);
  CHECK_EQ(chairfw::calLeft, 0);
  CHECK_EQ(chairfw::calId, savedCal);
  CHECK(chairfw::calibrated);
  CHECK(!chairfw::gate.alert && !bodyfw::alert);
  CHECK_EQ(bodyfw::chairLink.control(), CTL_CHAIR);
  CHECK(isnan(chairfw::body.pitch) && isnan(chairfw::body.roll));
  chairSerial.in = "c"; runFor(100);
  CHECK_EQ(chairfw::calLeft, 0);

  // Manual motor testing is still possible, explicitly distinct from posture.
  chairSerial.in = "m"; runFor(150);
  CHECK(bodyfw::testRunning(sim::now + BODY_CLOCK));
  CHECK(bodyfw::ledState && !bodyfw::alert);
  runFor(600);
  CHECK(!bodyfw::ledState);
  Wire1.forceStale = false;
  Wire1.reset(); runFor(1600);                 // brownout/config lost -> bounded retry -> recovery
  CHECK_EQ(bodyfw::imuStatus, IMU_OK);
  CHECK_EQ(chairfw::calId, savedCal);
  CHECK(!bodyfw::alert);

  tilt(20); runFor(3500);
  CHECK(chairfw::gate.alert && bodyfw::alert);
  bodyfw::radio.downOk = false;               // retain the pre-fault alert ACK
  Wire1.nackAll = true; runFor(50);
  CHECK_EQ(bodyfw::imuStatus, IMU_BUS);
  CHECK(!bodyfw::alert && !bodyfw::ledState);
  CHECK(!chairfw::gate.alert);
  Wire1.nackAll = false; runFor(350);
  CHECK_EQ(bodyfw::imuStatus, IMU_OK);
  CHECK(bodyfw::inhibited && !bodyfw::alert);  // a healthy vector cannot authorize an old decision
  bodyfw::radio.downOk = true; runFor(500);
  CHECK(!bodyfw::alert);
  runFor(3000);
  CHECK(bodyfw::alert && chairfw::gate.alert); // chair hold restarted from healthy samples

  bodyfw::radio.downOk = false; runFor(4400);
  CHECK_EQ(bodyfw::chairLink.control(), CTL_FALLBACK);
  CHECK(bodyfw::alert);
  Wire1.forceStale = true; runFor(100);
  CHECK(!bodyfw::alert && !bodyfw::ledState);
  CHECK_EQ(bodyfw::chairLink.ack.calId, savedCal);
  Wire1.forceStale = false; runFor(2900);
  CHECK(!bodyfw::alert);
  runFor(300);
  CHECK(bodyfw::alert);
  bodyfw::radio.downOk = true; runFor(150);
  CHECK_EQ(bodyfw::chairLink.control(), CTL_CHAIR);
  Wire1.useRaw = true; Wire1.raw[0] = Wire1.raw[1] = Wire1.raw[2] = 0;
  runFor(100);
  CHECK_EQ(bodyfw::imuStatus, IMU_RANGE);
  CHECK(!bodyfw::alert && !chairfw::gate.alert);
  CHECK_EQ(chairfw::calId, savedCal);
  Wire1.useRaw = false; tilt(0); runFor(200);
  CHECK_EQ(bodyfw::imuStatus, IMU_OK);
  for (const auto &bytes : bodyfw::radio.sent) {
    BodyPacket parsed;
    CHECK(parseBodyPacket(bytes.data(), bytes.size(), parsed));
  }

  // every decision the body reported is the one the chair row made
  c = rows(chairSerial.out);
  int joins = checkJoins(c);
  printf("  %d body reports joined to the chair row that decided them\n", joins);
  CHECK(joins > 300);
  CHECK(sim::now + BODY_CLOCK < BODY_CLOCK);   // the body's millis() wrapped during the run
  CHECK_EQ(chairSerial.blocked + bodySerial.blocked, 0);
  CHECK_EQ(chairfw::out.rowDrops() + bodyfw::out.rowDrops(), 0u);

  if (argc == 3) {
    FILE *f = fopen(argv[1], "wb"); fwrite(chairSerial.out.data(), 1, chairSerial.out.size(), f); fclose(f);
    f = fopen(argv[2], "wb"); fwrite(bodySerial.out.data(), 1, bodySerial.out.size(), f); fclose(f);
  }
  return report("system (chair.ino + body.ino)");
}
