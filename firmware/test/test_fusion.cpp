// Host tests for PostureLink/Fusion.h, the chair's IMU + FSR alert rule:
//   make -C firmware/test
// Unit cases, then a cross-check against analysis/fusion.py: real log excerpts
// replayed by the Python rule (fixtures/fusion_cases.txt, written by
// `analysis/fusion.py --fixture`) must give the same reason code and alert
// on every row.
#include <Fusion.h>

#include "check.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace posture;

static const float UPRIGHT[N_SPOTS] = {2400, 2700, 2900, 2600, 2300, 2400, 2800, 2600};

static Decision dec(float dp, float dr, const float *f = UPRIGHT, const float *base = UPRIGHT) {
  return decide(dp, dr, f, FsrBaseline::of(base));
}

static void testDecide() {
  Decision d = dec(1, -2);
  CHECK_EQ(d.reason, R_OK); CHECK(!d.bad); CHECK(d.ok);
  d = dec(8, 0);                               // between clear and alert: neither
  CHECK_EQ(d.reason, R_OK); CHECK(!d.bad); CHECK(!d.ok);
  CHECK_EQ(dec(NAN, 0).reason, R_INVALID);
  CHECK_EQ(dec(0, INFINITY).reason, R_INVALID);

  const float empty[N_SPOTS] = {0, 0, 0, 0, 2000, 2000, 2000, 2000};   // backrest alone: empty
  d = dec(30, 0, empty);
  CHECK_EQ(d.reason, R_EMPTY); CHECK(!d.bad); CHECK(!d.ok);

  float slid[N_SPOTS];
  memcpy(slid, UPRIGHT, sizeof slid);
  slid[SEAT_REAR_LEFT] = slid[SEAT_REAR_RIGHT] = 0; slid[SEAT_FRONT_LEFT] = 3000;
  d = dec(0, 4, slid);                         // small in the IMU, clear in the seat
  CHECK_EQ(d.reason, R_SLUMP); CHECK(d.bad); CHECK(!d.ok);

  d = dec(2, 20);                              // reclined: backward tilt accepted
  CHECK_EQ(d.reason, R_RECLINE_OK); CHECK(!d.bad); CHECK(d.ok);
  CHECK_EQ(dec(2, 5).reason, R_OK);            // reclined but small: plain OK
  float half[N_SPOTS];
  memcpy(half, UPRIGHT, sizeof half);
  half[SEAT_REAR_LEFT] = 1500; half[SEAT_REAR_RIGHT] = 1300;   // hips half forward
  CHECK_EQ(dec(2, 20, half).reason, R_IMU_DEV);
  CHECK_EQ(dec(0, -12).reason, R_IMU_DEV);     // forward never ignored
  CHECK_EQ(dec(12, 20).reason, R_IMU_DEV);     // nor lateral

  float left[N_SPOTS];
  memcpy(left, UPRIGHT, sizeof left);
  left[SEAT_FRONT_LEFT] = 3200; left[SEAT_REAR_LEFT] = 3500;
  left[SEAT_FRONT_RIGHT] = 2200; left[SEAT_REAR_RIGHT] = 2000;
  d = dec(-6, -3, left);
  CHECK_EQ(d.reason, R_LEAN); CHECK(d.bad);
  CHECK_EQ(dec(6, -3, left).reason, R_OK);     // seat moved the other way
  CHECK_EQ(dec(-4, -3, left).reason, R_OK);    // too small in the IMU

  // a ~0 baseline (calibrated with the seat rear unloaded) turns the seat terms off
  const float zero[N_SPOTS] = {2400, 2700, 0, 0, 0, 0, 2800, 2600};
  FsrBaseline b = FsrBaseline::of(zero);
  CHECK(!b.rearOk);
  CHECK_EQ(decide(0, 4, slid, b).reason, R_OK);
  CHECK_EQ(decide(0, 20, slid, b).reason, R_IMU_DEV);

  // FSR terms off == the IMU-only rule (PostureLink deviation/thresholds)
  FsrBaseline none;
  const float seated[N_SPOTS] = {3000, 3000, 3000, 3000, 0, 0, 0, 0};
  for (int i = 0; i < 200; i++) {
    float dp = (i % 20) - 10.0f, dr = (i / 10) - 10.0f;
    float dev = deviation(dp, dr, 0, 0);
    Decision x = decide(dp, dr, seated, none);
    CHECK_EQ(x.bad, dev > ALERT_DEG);
    CHECK_EQ(x.ok, dev < CLEAR_DEG);
  }
}

static void testGate() {
  FusionGate g;
  CHECK(!g.update(true, false, 1000));
  CHECK(!g.update(true, false, 3999));
  CHECK(g.update(true, false, 4000));          // changed: on after ALERT_HOLD_MS
  CHECK(g.alert);
  CHECK(!g.update(false, false, 4100));        // neither: held
  CHECK(g.alert);
  CHECK(g.update(false, true, 4200));          // ok clears
  CHECK(!g.alert);
  g.update(true, false, 5000);
  g.update(false, false, 7000);                // a non-bad sample restarts the hold
  CHECK(!g.update(true, false, 9000));
  CHECK(!g.update(true, false, 11999));
  CHECK(g.update(true, false, 12000));
  g.reset();
  CHECK(!g.alert); CHECK_EQ(g.overSince, 0ul);
}

// fixtures/fusion_cases.txt: B = baseline + gate state, S = one sample + expected result.
static void testPythonCrossCheck(const char *path) {
  FILE *fp = fopen(path, "r");
  CHECK(fp != nullptr);
  if (!fp) return;
  char line[256];
  FsrBaseline base;
  FusionGate gate;
  const unsigned long T0 = 100000;             // excerpt clock offset (never 0)
  int rows = 0, excerpts = 0, mismatches = 0, seen[7] = {}, edges = 0;
  bool prev = false;
  while (fgets(line, sizeof line, fp)) {
    if (line[0] == 'B') {
      float m[N_SPOTS];
      int alert, held;
      CHECK_EQ(sscanf(line + 1, "%f %f %f %f %f %f %f %f %d %d", &m[0], &m[1], &m[2], &m[3],
                      &m[4], &m[5], &m[6], &m[7], &alert, &held), 10);
      base = FsrBaseline::of(m);
      gate.reset();
      gate.alert = alert;
      if (held >= 0) gate.overSince = T0 - held;
      prev = gate.alert;
      excerpts++;
    } else if (line[0] == 'S') {
      unsigned long dt;
      float dp, dr, f[N_SPOTS];
      int reason, alert;
      CHECK_EQ(sscanf(line + 1, "%lu %f %f %f %f %f %f %f %f %f %f %d %d", &dt, &dp, &dr, &f[0],
                      &f[1], &f[2], &f[3], &f[4], &f[5], &f[6], &f[7], &reason, &alert), 13);
      // as chair.ino decideAlert(): EMPTY/INVALID reset the gate
      Decision d = decide(dp, dr, f, base);
      if (d.reason == R_EMPTY || d.reason == R_INVALID) gate.reset();
      else gate.update(d.bad, d.ok, T0 + dt);
      if (d.reason != reason || gate.alert != (alert != 0)) {
        if (mismatches++ < 10)
          printf("  mismatch row %d: dp=%.1f dr=%.1f -> reason %d alert %d, python %d %d\n",
                 rows, dp, dr, d.reason, gate.alert, reason, alert);
      }
      if (reason >= 0 && reason < 7) seen[reason]++;
      edges += gate.alert != prev;
      prev = gate.alert;
      rows++;
    }
  }
  fclose(fp);
  printf("  python cross-check: %d excerpts, %d rows, %d alert edges, %d mismatches; reasons",
         excerpts, rows, edges, mismatches);
  for (int r = 0; r < 7; r++) printf(" %d:%d", r, seen[r]);
  printf("\n");
  CHECK_EQ(mismatches, 0);
  CHECK(rows > 1000);
  CHECK(edges >= 5);                           // alerts switching on and off, not just held
  for (int r = R_OK; r <= R_EMPTY; r++) CHECK(seen[r] > 0);   // every reason but INVALID
}

int main(int argc, char **argv) {
  testDecide();
  testGate();
  testPythonCrossCheck(argc > 1 ? argv[1] : "fixtures/fusion_cases.txt");
  return report("test_fusion");
}
