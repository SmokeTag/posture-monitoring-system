// Host unit tests for firmware/libraries/PostureLink (no board needed):
//   make -C firmware/test
// Covers the body's link freshness (BodyLink), the chair's seq bookkeeping
// (SeqTracker) and the non-blocking serial output (Line, SerialOut).
#include <PostureLink.h>
#include <SerialOut.h>

#include "check.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

using namespace posture;

// ---------------------------------------------------------------- BodyLink

static ChairAck mkAck(uint8_t flags, uint8_t buzz = 0, float p = 1.5f, float r = -2.5f) {
  ChairAck a = {};
  a.ver = PROTO_VER; a.calId = 1;
  a.flags = flags; a.buzz = buzz; a.pitchRef = p; a.rollRef = r;
  return a;
}

static ChairAck based(uint8_t boot, uint16_t seq, uint8_t flags = CF_CAL) {
  ChairAck a = mkAck(flags);
  a.basisBoot = boot; a.basisSeq = seq;
  return a;
}

const uint8_t BOOT = 0x42;

// Feed one transmission (sample number = writes so far); returns the result.
static BodyLink::Result tx(BodyLink &L, uint32_t t, bool ok, uint8_t arc, const ChairAck *a,
                           uint8_t len = sizeof(ChairAck)) {
  return L.onTx(t, L.sent, BOOT, ok, arc, a, a ? len : 0);
}

// One write of sample n with ACK `a` (always ok).
static void txn(BodyLink &L, uint32_t n, const ChairAck &a, uint8_t boot = BOOT) {
  L.onTx(n * 50, n, boot, true, 0, &a, sizeof a);
}

static void testParseChairAck() {
  ChairAck out = mkAck(0, 9);
  ChairAck good = mkAck(CF_CAL | CF_ALERT, 3);
  CHECK(parseChairAck(&good, sizeof good, out));
  CHECK_EQ(out.buzz, 3);
  CHECK(!parseChairAck(&good, 0, out));                       // empty ACK
  CHECK(!parseChairAck(&good, sizeof good - 1, out));         // short
  uint8_t big[32] = {};
  CHECK(!parseChairAck(big, sizeof big, out));                // long
  ChairAck unk = mkAck(0x80);
  CHECK(!parseChairAck(&unk, sizeof unk, out));               // unknown flag bit
  ChairAck nanRef = mkAck(CF_CAL, 0, NAN, 0);
  CHECK(!parseChairAck(&nanRef, sizeof nanRef, out));         // CAL with a NaN reference
  ChairAck infRef = mkAck(CF_CAL, 0, 0, INFINITY);
  CHECK(!parseChairAck(&infRef, sizeof infRef, out));
  ChairAck farRef = mkAck(CF_CAL, 0, 90.5f, 0);               // no accelerometer pitch is > 90
  CHECK(!parseChairAck(&farRef, sizeof farRef, out));
  ChairAck edgeRef = mkAck(CF_CAL, 0, -90, 180);
  CHECK(parseChairAck(&edgeRef, sizeof edgeRef, out));
  ChairAck ver1 = mkAck(CF_CAL); ver1.ver = 1;                // another protocol version
  CHECK(!parseChairAck(&ver1, sizeof ver1, out));
  ChairAck noId = mkAck(CF_CAL); noId.calId = 0;              // calId 0 is "none"
  CHECK(!parseChairAck(&noId, sizeof noId, out));
  ChairAck alertNoCal = mkAck(CF_ALERT);                      // alert without a reference
  CHECK(!parseChairAck(&alertNoCal, sizeof alertNoCal, out));
  ChairAck seqNoBasis = mkAck(CF_CAL); seqNoBasis.basisSeq = 5;
  CHECK(!parseChairAck(&seqNoBasis, sizeof seqNoBasis, out));
  // a v1 ChairAck (flags, buzz, reserved, refs: 12 bytes) is never taken
  uint8_t v1[12] = {CF_CAL | CF_ALERT, 3, 0, 0};
  CHECK(!parseChairAck(v1, sizeof v1, out));
  ChairAck nanNoCal = mkAck(0, 0, NAN, NAN);                  // refs unused without CAL
  CHECK(parseChairAck(&nanNoCal, sizeof nanNoCal, out));
  CHECK_EQ(out.buzz, 0);                                      // failures above left `out` alone
}

static BodyPacket mkPkt(uint8_t ctl, uint8_t flags, uint8_t calId, uint8_t age, uint16_t seq = 100) {
  BodyPacket p = {};
  p.ver = PROTO_VER; p.boot = 0x42; p.seq = seq; p.pitch = 10; p.roll = -20;
  p.ctl = ctl; p.flags = flags; p.calId = calId; p.age = age;
  p.ackSeq = age == AGE_NONE ? 0 : (uint16_t)(seq - (age < AGE_MAX ? age : 300));
  return p;
}

static void testParseBodyPacket() {
  BodyPacket out = {};
  BodyPacket good = mkPkt(CTL_CHAIR, BF_CAL | BF_ALERT | BF_MOTOR, 7, 2);
  CHECK(parseBodyPacket(&good, sizeof good, out));
  CHECK_EQ(out.ackSeq, 98);
  BodyPacket wrap = mkPkt(CTL_CHAIR, BF_CAL, 7, 2, 1);        // ackSeq 65535: across the wrap
  CHECK(parseBodyPacket(&wrap, sizeof wrap, out));
  CHECK_EQ(out.ackSeq, 65535);
  BodyPacket stale = mkPkt(CTL_FALLBACK, BF_CAL, 7, AGE_MAX); // age 254+: ackSeq not checked
  CHECK(parseBodyPacket(&stale, sizeof stale, out));
  BodyPacket none = mkPkt(CTL_NONE, BF_MOTOR | BF_TEST, 0, AGE_NONE);   // test buzz, no chair yet
  CHECK(parseBodyPacket(&none, sizeof none, out));
  out = {};
  CHECK(!parseBodyPacket(&good, sizeof good - 1, out));       // size
  uint8_t v1[12] = {5, 0, 0, 0x5a};                           // v1: seq, flags, boot, pitch, roll
  CHECK(!parseBodyPacket(v1, sizeof v1, out));
  BodyPacket p = good; p.ver = 3;
  CHECK(!parseBodyPacket(&p, sizeof p, out));
  p = good; p.boot = 0;
  CHECK(!parseBodyPacket(&p, sizeof p, out));
  p = good; p.pitch = NAN;
  CHECK(!parseBodyPacket(&p, sizeof p, out));
  p = good; p.roll = -180.5f;
  CHECK(!parseBodyPacket(&p, sizeof p, out));
  p = good; p.ctl = 3;
  CHECK(!parseBodyPacket(&p, sizeof p, out));
  p = good; p.flags = 0x10;
  CHECK(!parseBodyPacket(&p, sizeof p, out));
  p = good; p.flags = BF_ALERT;                               // alert with no reference
  CHECK(!parseBodyPacket(&p, sizeof p, out));
  p = good; p.calId = 0;                                      // chair/fallback hold an id
  CHECK(!parseBodyPacket(&p, sizeof p, out));
  p = none; p.calId = 3;                                      // none holds nothing
  CHECK(!parseBodyPacket(&p, sizeof p, out));
  p = none; p.age = 2; p.ackSeq = 98;
  CHECK(!parseBodyPacket(&p, sizeof p, out));
  p = good; p.ackSeq = 97;                                    // age doesn't match seq - ackSeq
  CHECK(!parseBodyPacket(&p, sizeof p, out));
  p = good; p.age = AGE_NONE;                                 // unknown age with an ackSeq
  CHECK(!parseBodyPacket(&p, sizeof p, out));
  CHECK_EQ(out.seq, 0);                                       // failures left `out` alone
}

// Age of the held ChairAck, in body samples (BodyLink::age).
static void testAge() {
  // decided on sample n-1, arrives with the ACK of n: age 1, then grows
  BodyLink L;
  CHECK_EQ(L.age(0), AGE_NONE);
  CHECK_EQ(L.control(), CTL_NONE);
  txn(L, 10, based(BOOT, 9));
  CHECK_EQ(L.age(10), 1);
  CHECK_EQ(L.age(11), 2);                                     // what packet 11 reports
  CHECK_EQ(L.control(), CTL_CHAIR);
  // age 0: the chair got the first copy of 11, its ACK was lost, a retry of
  // 11 brought the decision made on 11 itself
  txn(L, 11, based(BOOT, 11));
  CHECK_EQ(L.age(11), 0);
  // the same basis repeated doesn't reset the count
  for (uint32_t n = 12; n < 40; n++) txn(L, n, based(BOOT, 11));
  CHECK_EQ(L.age(39), 28);
  // at most SEQ_MAX_GAP behind: accepted; one more: unknown
  BodyLink G;
  txn(G, 1000, based(BOOT, 1000 - SEQ_MAX_GAP));
  CHECK_EQ(G.age(1000), SEQ_MAX_GAP);
  BodyLink H;
  txn(H, 1000, based(BOOT, 1000 - SEQ_MAX_GAP - 1));
  CHECK_EQ(H.age(1000), AGE_NONE);
  // a basis can't be before this boot's first sample
  BodyLink E;
  txn(E, 3, based(BOOT, 65535));
  CHECK_EQ(E.age(3), AGE_NONE);
  // another boot id: not this session's sample
  BodyLink B;
  txn(B, 100, based(0x43, 99));
  CHECK_EQ(B.age(100), AGE_NONE);
  CHECK(B.watch.up);                                          // still control: the link is up
  // across the seq wrap: n = 65537 is seq 1, basis 65535 -> 2 samples
  BodyLink W;
  txn(W, 65537, based(BOOT, 65535));
  CHECK_EQ(W.age(65537), 2);
  // a ChairAck without basis replaces the held one: unknown, not the old age
  txn(W, 65538, based(0, 0));
  CHECK_EQ(W.age(65538), AGE_NONE);
  txn(W, 65539, based(BOOT, 2));                              // a new decision: known again
  CHECK_EQ(W.age(65539), 1);
  // long fallback: no ChairAck for > 65536 samples -> saturates at AGE_MAX
  BodyLink F;
  txn(F, 5, based(BOOT, 4));
  CHECK_EQ(F.age(5 + 253), 254);
  CHECK_EQ(F.age(5 + 300), AGE_MAX);
  CHECK_EQ(F.age(5 + 65536 + 1), AGE_MAX);                    // seq is back near 5: still old
  // ...and the same basis coming back after the wrap is still that old one
  txn(F, 5 + 65536, based(BOOT, 4));
  CHECK_EQ(F.age(5 + 65536), AGE_MAX);
  // an invalid basis repeated until the seq wraps doesn't turn valid either
  BodyLink R;
  txn(R, 200, based(BOOT, 100));                              // 100 behind: rejected
  CHECK_EQ(R.age(200), AGE_NONE);
  txn(R, 100 + 65536 + 1, based(BOOT, 100));                  // u16 delta 1 now
  CHECK_EQ(R.age(100 + 65536 + 1), AGE_NONE);
}

static void testFreshnessNeedsValidAck() {
  BodyLink L;
  ChairAck cal = mkAck(CF_CAL, 0, 4.0f, -1.0f);
  // no ACK payload yet: transport alone never brings the link up
  CHECK(!tx(L, 0, true, 0, nullptr).linkChanged);
  CHECK(!L.watch.up);
  CHECK(!L.haveAck);
  // first valid ChairAck -> up
  auto r = tx(L, 50, true, 0, &cal);
  CHECK(r.ctl && r.linkChanged && L.watch.up);
  // valid ACK then a stream of empty / wrong-size / invalid ACKs: the radio is
  // ACKing, but no control arrives -> must time out 1 s after the last valid one
  ChairAck unk = mkAck(0x40);
  uint32_t t = 100;
  for (; t < 50 + LINK_TIMEOUT_MS; t += 50) {
    const ChairAck *a = (t / 50) % 3 == 0 ? nullptr : &unk;
    uint8_t len = (t / 50) % 3 == 1 ? sizeof(ChairAck) - 2 : sizeof(ChairAck);
    r = tx(L, t, true, 1, a, len);
    CHECK(!r.ctl && !r.linkChanged);
    CHECK(L.watch.up);
  }
  r = tx(L, t, true, 0, nullptr);                              // t = 1050: 1000 ms since the last valid
  CHECK(r.linkChanged && !L.watch.up);
  // fallback keeps the last valid reference (not the invalid ones)
  CHECK(L.haveAck);
  CHECK(L.ack.flags & CF_CAL);
  CHECK(L.ack.pitchRef == 4.0f && L.ack.rollRef == -1.0f);
  CHECK_EQ(L.sent, 22u);
  CHECK_EQ(L.acked, 22u);
  CHECK_EQ(L.ctl, 1u);
  CHECK_EQ(L.bad, 21u);
}

static void testTxFailuresAndRecovery() {
  BodyLink L;
  ChairAck a0 = mkAck(CF_CAL, 7);
  tx(L, 0, true, 2, &a0);
  CHECK(L.watch.up);
  // radio failures (all retries spent: ARC = 15) for 1 s -> down
  uint32_t t = 50;
  for (; t < LINK_TIMEOUT_MS; t += 50) { tx(L, t, false, 15, nullptr); CHECK(L.watch.up); }
  CHECK(tx(L, t, false, 15, nullptr).linkChanged);
  CHECK(!L.watch.up);
  CHECK_EQ(L.sent, 21u);
  CHECK_EQ(L.acked, 1u);                                       // noack = 20
  CHECK_EQ(L.retx, 2u + 20u * 15u);                            // failed writes' retries count too
  // the chair asked for a buzz while we were away: reconnecting only syncs it
  ChairAck a1 = mkAck(CF_CAL, 8);
  auto r = tx(L, t + 50, true, 0, &a1);
  CHECK(r.linkChanged && L.watch.up && !r.buzz);
  // a new request while linked buzzes once
  ChairAck a2 = mkAck(CF_CAL, 9);
  r = tx(L, t + 100, true, 0, &a2);
  CHECK(r.buzz && !r.linkChanged);
  r = tx(L, t + 150, true, 0, &a2);
  CHECK(!r.buzz);
  // a chair that clears its calibration is believed (refs dropped from the fallback)
  ChairAck a3 = mkAck(0, 9);
  tx(L, t + 200, true, 0, &a3);
  CHECK(!(L.ack.flags & CF_CAL));
  L.clearCounts();
  CHECK_EQ(L.sent + L.acked + L.retx + L.ctl + L.bad, 0u);
}

static void testFreshnessRollover() {
  BodyLink L;
  ChairAck a = mkAck(0);
  uint32_t t0 = 0xFFFFFFFFu - 120;                             // millis() about to wrap
  tx(L, t0, true, 0, &a);
  CHECK(L.watch.up);
  uint32_t t = t0;
  for (int i = 0; i < 19; i++) { t += 50; tx(L, t, true, 0, nullptr); CHECK(L.watch.up); }
  t += 50;                                                     // wrapped; 1000 ms after t0
  CHECK(t < t0);
  CHECK(tx(L, t, true, 0, nullptr).linkChanged);
  CHECK(!L.watch.up);
  // once down, an elapsed time that wraps all the way round can't revive it
  CHECK(!tx(L, t0 + 10, true, 0, nullptr).linkChanged);
  CHECK(!L.watch.up);
  // and a valid ACK across the wrap keeps it up
  BodyLink M;
  tx(M, t0, true, 0, &a);
  tx(M, t0 + 900, true, 0, &a);                                // wraps
  CHECK(!tx(M, t0 + 1800, true, 0, nullptr).linkChanged);
  CHECK(M.watch.up);
  CHECK(tx(M, t0 + 1900, true, 0, nullptr).linkChanged);
}

// -------------------------------------------------------------- SeqTracker

static void testSeqNormalAndWrap() {
  SeqTracker S;
  uint32_t t = 0;
  CHECK_EQ(S.onPacket(7, 65530, t), SEQ_FIRST);
  for (uint16_t s = 65531; s != 4; s++) { t += 50; CHECK_EQ(S.onPacket(7, s, t), SEQ_NEXT); }
  CHECK_EQ(S.pkts, 10u);                                       // 65530..65535, 0..3
  CHECK_EQ(S.lost, 0u);                                        // 65535 -> 0 is one step
  CHECK_EQ(S.rst, 0u);
  // real gap while linked (also across the wrap)
  SeqTracker G;
  G.onPacket(7, 65534, 0);
  CHECK_EQ(G.onPacket(7, 2, 200), SEQ_NEXT);                   // 65535, 0, 1 missing
  CHECK_EQ(G.lost, 3u);
  CHECK_EQ(G.onPacket(7, 5, 400), SEQ_NEXT);
  CHECK_EQ(G.lost, 5u);
}

static void testSeqReboot() {
  // body rebooted at seq 1000 (the old code counted 64535 lost)
  SeqTracker S;
  S.onPacket(0x11, 999, 0);
  S.onPacket(0x11, 1000, 50);
  CHECK_EQ(S.onPacket(0x22, 0, 2500), SEQ_RESTART);            // new boot id, after an outage
  CHECK_EQ(S.lost, 0u);
  CHECK_EQ(S.rst, 1u);
  CHECK_EQ(S.onPacket(0x22, 1, 2550), SEQ_NEXT);
  CHECK_EQ(S.lost, 0u);
  // even a fast reboot (still "linked") with a new boot id
  SeqTracker F;
  F.onPacket(0x11, 40000, 0);
  CHECK_EQ(F.onPacket(0x33, 40001, 50), SEQ_RESTART);          // same seq region, new boot
  CHECK_EQ(F.lost, 0u);
  // old body firmware (boot = 0): inferred from seq going back
  SeqTracker O;
  O.onPacket(0, 1000, 0);
  CHECK_EQ(O.onPacket(0, 0, 2000), SEQ_RESTART);
  CHECK_EQ(O.onPacket(0, 1, 2050), SEQ_NEXT);
  CHECK_EQ(O.onPacket(0, 30, 3100), SEQ_RESUME);              // forward after an outage
  CHECK_EQ(O.onPacket(0, 3, 3150), SEQ_RESTART);               // back, boot unknown: no "old" guess
  CHECK_EQ(O.lost, 0u);
  // boot id collision (1/255) after an outage: seq went back -> restart
  SeqTracker C;
  C.onPacket(5, 300, 0);
  CHECK_EQ(C.onPacket(5, 0, 3000), SEQ_RESTART);
  CHECK_EQ(C.lost, 0u);
}

static void testSeqDupOldAndJumps() {
  SeqTracker S;
  S.onPacket(9, 100, 0);
  S.onPacket(9, 101, 50);
  CHECK_EQ(S.onPacket(9, 101, 60), SEQ_DUP);
  CHECK_EQ(S.onPacket(9, 99, 70), SEQ_OLD);                    // slightly behind, linked
  CHECK_EQ(S.dup, 1u);
  CHECK_EQ(S.old, 1u);
  CHECK_EQ(S.pkts, 2u);                                        // dup/old are not samples
  CHECK_EQ(S.onPacket(9, 102, 100), SEQ_NEXT);                 // ... and don't disturb the sequence
  CHECK_EQ(S.lost, 0u);
  // far behind while linked, same boot: not a plausible reorder -> restart
  CHECK_EQ(S.onPacket(9, 2, 150), SEQ_RESTART);
  // unexplained forward jump while linked -> restart, not "lost"
  CHECK_EQ(S.onPacket(9, 2 + SEQ_MAX_GAP + 1, 200), SEQ_RESTART);
  CHECK_EQ(S.onPacket(9, 2 + SEQ_MAX_GAP + 1 + SEQ_MAX_GAP, 250), SEQ_NEXT);   // max gap still counts
  CHECK_EQ(S.lost, (uint32_t)SEQ_MAX_GAP - 1);
  CHECK_EQ(S.rst, 2u);
}

static void testSeqReconnect() {
  SeqTracker S;
  S.onPacket(4, 10, 0);
  S.onPacket(4, 11, 50);
  // 5 s outage, body kept sending: resumed, the ~100 missed packets are not "lost"
  CHECK_EQ(S.onPacket(4, 111, 5050), SEQ_RESUME);
  CHECK_EQ(S.lost, 0u);
  CHECK_EQ(S.rst, 0u);
  CHECK_EQ(S.onPacket(4, 113, 5150), SEQ_NEXT);
  CHECK_EQ(S.lost, 1u);
  // gap just under the timeout is still "linked"
  CHECK_EQ(S.onPacket(4, 120, 5150 + LINK_TIMEOUT_MS - 1), SEQ_NEXT);
  CHECK_EQ(S.lost, 7u);
  // millis() rollover between packets
  SeqTracker R;
  R.onPacket(4, 1, 0xFFFFFFF0u);
  CHECK_EQ(R.onPacket(4, 3, 0x20u), SEQ_NEXT);
  CHECK_EQ(R.lost, 1u);
  S.clearCounts();
  CHECK_EQ(S.pkts + S.lost + S.dup + S.old + S.rst, 0u);
}

// --------------------------------------------------------------------- Line

static std::string fmt(float v, int d) {
  char b[64];
  snprintf(b, sizeof b, "%.*f", d, (double)v);
  return b;
}

static void testLineFormat() {
  Line l;
  l.u32(0).chr(',').u32(4294967295u).chr(',').i32(-1).chr(',').i32(-2147483647 - 1)
   .chr(',').hex8(0x0a).chr(',').str("x");
  CHECK_STR(l.text(), "0,4294967295,-1,-2147483648,0a,x");
  CHECK_EQ(l.size(), l.length() + 2);
  CHECK(memcmp(l.bytes() + l.length(), "\r\n", 2) == 0);

  // same text as the core's Print::print(float, d) = snprintf("%.*f")
  const float fixed[] = {0.0f, -0.0f, 0.04f, -0.04f, 0.05f, 0.15f, 0.25f, 0.35f, 2.5f, 3.5f,
                         -2.5f, 9.95f, 99.95f, 1685887.5f, 1685888.5f, -1.0f, 179.99f,
                         -180.0f, 1e9f, 123456.789f};
  for (float v : fixed)
    for (int d = 0; d <= 2; d++) { Line x; x.fix(v, d); CHECK_STR(x.text(), fmt(v, d)); }
  srand(1);
  for (int i = 0; i < 200000; i++) {
    float v = ((float)rand() / RAND_MAX - 0.5f) * (i % 2 ? 400.0f : 4.0e6f);
    if (i % 7 == 0) v = (float)(rand() % 20000) / 4.0f;   // exact .25/.5/.75 ties
    int d = i % 3 == 0 ? 0 : 1;
    Line x; x.fix(v, d);
    if (std::string(x.text()) != fmt(v, d)) {
      CHECK_STR(x.text(), fmt(v, d));
      break;
    }
  }
  Line n; n.fix(NAN, 1).chr(',').fix(INFINITY, 1).chr(',').fix(-INFINITY, 0);
  CHECK_STR(n.text(), "nan,inf,-inf");

  // too long: marked bad, never truncated silently
  Line big;
  for (int i = 0; i < Line::CAP; i++) big.chr('a');
  CHECK(big.ok());
  big.chr('b');
  CHECK(!big.ok());
  CHECK_EQ(big.length(), Line::CAP);
}

// ---------------------------------------------------------------- SerialOut

// Stand-in for Adafruit_USBD_CDC: a FIFO of `cap` bytes that only drains when
// the "host" reads. write() asserts the no-block contract.
struct FakePort {
  bool dtrOn = false;
  size_t cap = 256, used = 0;
  std::string got;                             // what the host received
  int writes = 0, blocked = 0;
  int dtr() { return dtrOn; }
  int availableForWrite() { return (int)(cap - used); }
  size_t write(const uint8_t *b, size_t n) {
    writes++;
    if (!dtrOn) return 0;                      // real write(): loop doesn't run when disconnected
    if (used + n > cap) { blocked++; return 0; }   // real write() would spin here
    used += n;
    got.append((const char *)b, n);
    return n;
  }
  void hostReads() { used = 0; }
};

static int hdrLines = 3;
static bool fakeHeader(uint8_t i, Line &l) {
  if (i == 1) return true;                     // an absent init failure: skipped
  if (i >= hdrLines) return false;
  l.str("H").u32(i);
  return true;
}

static Line mk(const char *s) { Line l; l.str(s); return l; }

static std::vector<std::string> lines(const std::string &s) {
  std::vector<std::string> v;
  size_t p = 0, e;
  while ((e = s.find("\r\n", p)) != std::string::npos) { v.push_back(s.substr(p, e - p)); p = e + 2; }
  CHECK_EQ(p, s.size());                       // nothing partial
  return v;
}

static void testSerialNoHost() {
  FakePort P;
  SerialOut<FakePort> out(P, fakeHeader);
  out.poll();
  CHECK(out.row(mk("1,2")));                   // discarded, not counted: no reader
  out.event(mk("# e"));
  CHECK(out.stats(mk("# rx")));
  CHECK_EQ(P.writes, 0);
  CHECK_EQ(out.rowDrops() + out.eventDrops(), 0u);
}

static void testSerialHeaderOnConnect() {
  FakePort P;
  SerialOut<FakePort> out(P, fakeHeader);
  out.event(mk("# before"));                   // no host yet: gone
  P.dtrOn = true;
  out.poll();
  CHECK(out.row(mk("10,1")));
  out.event(mk("# ev"));
  CHECK(out.row(mk("20,1")));
  auto v = lines(P.got);
  CHECK_EQ(v.size(), 5u);
  CHECK_STR(v[0], "H0");
  CHECK_STR(v[1], "H2");                       // H1 skipped (empty)
  CHECK_STR(v[2], "10,1");
  CHECK_STR(v[3], "# ev");
  CHECK_STR(v[4], "20,1");
  // re-request from line 2 (chair 'u')
  out.header(2);
  out.poll();
  CHECK_STR(lines(P.got).back(), "H2");
  // host closes and reopens: header again, drop counts restart
  P.dtrOn = false; out.poll();
  P.dtrOn = true; P.got.clear(); P.hostReads(); out.poll();
  v = lines(P.got);
  CHECK_EQ(v.size(), 2u);
  CHECK_STR(v[0], "H0");
}

static void testSerialStalledHost() {
  FakePort P;
  P.dtrOn = true;
  SerialOut<FakePort> out(P, fakeHeader);
  out.poll();
  P.hostReads(); P.got.clear();
  std::string row(126, '7');                   // ~chair row size; 2 x (126 + CRLF) = 256
  // host stops reading: the FIFO fills, then everything returns at once
  int sent = 0;
  for (int i = 0; i < 1000; i++) sent += out.row(mk(row.c_str()));
  CHECK_EQ(sent, 2);
  CHECK_EQ(P.availableForWrite(), 0);
  CHECK_EQ(out.rowDrops(), 998u);
  for (int i = 0; i < 20; i++) { Line l; l.str("# ev").u32(i); out.event(l); }
  CHECK_EQ(out.eventDrops(), 12u);             // 8 queued, the rest dropped
  CHECK(!out.stats(mk("# rx pkts=1")));        // not sent, not counted: caller retries
  CHECK_EQ(out.rowDrops(), 998u);
  CHECK_EQ(P.blocked, 0);                      // write() was never asked to wait
  // host resumes: queued events first (in order), then new rows
  P.hostReads();
  out.poll();
  P.hostReads();
  out.poll();
  CHECK(out.row(mk("after")));
  auto v = lines(P.got);
  CHECK_EQ(v.size(), 2u + 8u + 1u);
  for (int i = 0; i < 8; i++) CHECK_STR(v[2 + i], "# ev" + std::to_string(i));
  CHECK_STR(v.back(), "after");
  CHECK(out.stats(mk("# rx")));
  out.clearDrops();
  CHECK_EQ(out.rowDrops() + out.eventDrops(), 0u);
  CHECK_EQ(P.blocked, 0);
}

static void testSerialRowsWaitForEventsAndHeader() {
  FakePort P;
  P.cap = 20;                                  // tiny FIFO: header line fits, little else
  SerialOut<FakePort> out(P, fakeHeader);
  P.dtrOn = true;
  out.poll();                                  // H0 (4 bytes), H2 (4)
  out.event(mk("# 0123456789"));               // 14 bytes: doesn't fit behind the header
  CHECK(!out.row(mk("r1")));                   // must not overtake the queued event
  CHECK_EQ(out.rowDrops(), 1u);
  P.hostReads();
  CHECK(out.row(mk("r2")));
  auto v = lines(P.got);
  CHECK_EQ(v.size(), 4u);
  CHECK_STR(v[2], "# 0123456789");
  CHECK_STR(v[3], "r2");
  CHECK_EQ(P.blocked, 0);
  // too-long (bad) lines are counted, not cut
  Line bad;
  for (int i = 0; i <= Line::CAP; i++) bad.chr('y');
  uint32_t e0 = out.eventDrops();
  out.event(bad);
  CHECK_EQ(out.eventDrops(), e0 + 1);
}

int main() {
  testParseChairAck();
  testParseBodyPacket();
  testAge();
  testFreshnessNeedsValidAck();
  testTxFailuresAndRecovery();
  testFreshnessRollover();
  testSeqNormalAndWrap();
  testSeqReboot();
  testSeqDupOldAndJumps();
  testSeqReconnect();
  testLineFormat();
  testSerialNoHost();
  testSerialHeaderOnConnect();
  testSerialStalledHost();
  testSerialRowsWaitForEventsAndHeader();
  return report("PostureLink");
}
