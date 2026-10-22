"""Tests for captures.py (loader) and fusion.py (the IMU + FSR alert rule).

    QT_QPA_PLATFORM=offscreen .venv/bin/python -m unittest discover -s analysis
"""

from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

import captures as C
import fusion as F

COLS = ("t_ms,cal,calid,dev,alert,dpitch,droll,pitch,roll,boot,seq,bctl,bcal,bcalid,balert,"
        "bmotor,btest,bseq,bage,imu,binh,vbat,c8,c9,c10,c11,c12,c13,c14,c15,tag")
FSR_MAP = ("# fsr map c8=seat_front_left c9=seat_front_right c10=seat_rear_left "
           "c11=seat_rear_right c12=back_lower_right c13=back_upper_right "
           "c14=back_lower_left c15=back_upper_left")


def row(t_ms, calid, dp, dr, fsr, tag, cal=1, imu=0):
    dev = math.hypot(dp, dr)
    return (f"{t_ms},{cal},{calid},{dev:.1f},0,{dp},{dr},{dp},{dr},7,{t_ms // 50},1,1,{calid},0,0,0,"
            f"0,2,{imu},0,3900," + ",".join(str(v) for v in fsr) + f",{tag}")


def synthetic() -> str:
    """60 rows at 20 Hz: calid 5 (before logging), a capture -> calid 6, then tags."""
    lines = ["# tag map 0=transition 1=ereto 4=escorregado", FSR_MAP, COLS]
    t = 1000
    for i in range(60):
        if i == 10:
            lines.append("# calibrating (hold upright 1 s), from serial")
        if i == 29:
            lines.append("# calibrated upright: pitch=1.0 roll=2.0 calid=6")
        calid = 6 if i >= 29 else 5
        fsr = [100 * (i if 10 <= i < 30 else 1)] * 8   # capture rows 10..29 carry 100*i
        tag = 0 if i < 30 else (1 if i < 50 else 4)
        lines.append(row(t, calid, 0.0, 0.0, fsr, tag))
        t += 50
    return "\n".join(lines) + "\n"


class LoaderTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "2026-10-21_Some-One_run3.csv"
        self.path.write_text(synthetic())
        self.run = C.load(self.path)

    def tearDown(self):
        self.dir.cleanup()

    def test_header(self):
        r = self.run
        self.assertEqual((r.subject, r.run), ("Some-One", 3))
        self.assertEqual(r.fsr_map["seat_rear_left"], "c10")
        self.assertEqual(r.fsr_map["back_lower_left"], "c14")
        self.assertEqual(r.tags[4], "escorregado")
        self.assertEqual(list(r.fsr().columns), C.SPOTS)
        self.assertEqual(len(r.rows), 60)

    def test_calibration_window(self):
        """19 rows before the event + the first row after it (which has the new calid)."""
        pre, cal = self.run.cals
        self.assertFalse(pre.in_file)
        self.assertEqual((pre.calid, pre.start, pre.stop), (5, 0, C.CAL_SAMPLES))
        self.assertTrue(cal.in_file)
        self.assertEqual((cal.calid, cal.row, cal.start, cal.stop), (6, 29, 10, 30))
        self.assertEqual((cal.pitch, cal.roll), (1.0, 2.0))
        base = self.run.fsr_baseline(cal)
        self.assertAlmostEqual(base["seat_rear_left"], 100 * np.mean(range(10, 30)))

    def test_segments_and_settled(self):
        r = self.run.rows
        segs = C.segments(r)
        self.assertEqual(list(segs["tag"]), [1, 4])
        self.assertEqual(list(segs["first"]), [30, 50])
        # 2 s = 40 rows: the 20-row ereto segment and 10-row escorregado never settle
        self.assertFalse(r["settled"].any())
        st = C.settled(r, skip_s=0.5)
        self.assertEqual(int(st[30:50].sum()), 10)
        self.assertFalse(st[:30].any())

    def test_real_calibration_windows(self):
        """On every capture the window's IMU mean reproduces the logged reference."""
        for run in C.runs():
            for cal in run.cals:
                if not cal.in_file:
                    continue
                w = run.rows.iloc[cal.start:cal.stop]
                self.assertEqual(int(w["calid"].iloc[-1]), cal.calid)
                self.assertNotIn(cal.calid, set(w["calid"].iloc[:-1]))
                self.assertAlmostEqual(w["pitch"].mean(), cal.pitch, delta=0.06, msg=run.name)
                self.assertAlmostEqual(w["roll"].mean(), cal.roll, delta=0.06, msg=run.name)


UPRIGHT = [2400, 2700, 2900, 2600, 2300, 2400, 2800, 2600]   # SPOTS order, raw


def fsr(**kw):
    f = list(UPRIGHT)
    for k, v in kw.items():
        f[C.SPOTS.index(k)] = v
    return f


class RuleTest(unittest.TestCase):
    p = F.FUSION
    base = F.Baseline.of(UPRIGHT)

    def decide(self, dp, dr, f=UPRIGHT, base=None):
        return F.decide(dp, dr, f, base or self.base, self.p)

    def test_upright(self):
        self.assertEqual(self.decide(1.0, -2.0), (F.OK, False, True))

    def test_empty_and_invalid(self):
        self.assertEqual(self.decide(30.0, 0.0, [0] * 8)[0], F.EMPTY)
        self.assertEqual(self.decide(math.nan, math.nan)[0], F.INVALID)

    def test_slump_from_seat_rear(self):
        f = fsr(seat_rear_left=0, seat_rear_right=0, seat_front_left=3000)
        self.assertEqual(self.decide(0.0, 4.0, f), (F.SLUMP, True, False))   # small in the IMU

    def test_recline_ignores_backward_tilt(self):
        self.assertEqual(self.decide(2.0, 20.0)[:2], (F.RECLINE_OK, False))
        # hips half forward: backward tilt counts again
        f = fsr(seat_rear_left=1500, seat_rear_right=1300)
        self.assertEqual(self.decide(2.0, 20.0, f)[:2], (F.IMU_DEV, True))
        # forward and lateral deviation are never ignored
        self.assertEqual(self.decide(0.0, -12.0)[0], F.IMU_DEV)
        self.assertEqual(self.decide(12.0, 20.0)[0], F.IMU_DEV)

    def test_lean_needs_seat_shift_towards_it(self):
        left = fsr(seat_front_left=3200, seat_rear_left=3500, seat_front_right=2200, seat_rear_right=2000)
        self.assertEqual(self.decide(-6.0, -3.0, left)[:2], (F.LEAN, True))
        self.assertEqual(self.decide(6.0, -3.0, left)[:2], (F.OK, False))    # shift the other way
        self.assertEqual(self.decide(-4.0, -3.0, left)[:2], (F.OK, False))   # too small in the IMU

    def test_zero_baseline_falls_back_to_imu(self):
        base = F.Baseline.of([2400, 2700, 0, 0, 0, 0, 2800, 2600])
        self.assertFalse(base.rear_ok)
        f = fsr(seat_rear_left=0, seat_rear_right=0)
        self.assertEqual(self.decide(0.0, 4.0, f, base)[:2], (F.OK, False))   # no SLUMP
        self.assertEqual(self.decide(0.0, 20.0, f, base)[:2], (F.IMU_DEV, True))  # no recline

    def test_current_is_firmware_rule(self):
        rng = np.random.default_rng(1)
        for dp, dr in rng.uniform(-15, 15, (500, 2)):
            _, bad, ok = F.decide(dp, dr, [0] * 8, self.base, F.CURRENT)
            dev = math.hypot(dp, dr)
            self.assertEqual((bad, ok), (dev > 10, dev < 7))

    def test_gate(self):
        g = F.Gate(3.0)
        self.assertFalse(g.update(True, False, 0.0))
        self.assertFalse(g.update(True, False, 2.95))
        self.assertTrue(g.update(True, False, 3.0))
        self.assertTrue(g.update(False, False, 3.1))     # between thresholds: held
        self.assertFalse(g.update(False, True, 3.2))     # ok clears
        g.update(True, False, 4.0)
        self.assertFalse(g.update(True, False, 6.9))     # a non-bad sample restarts the hold
        g.update(False, False, 7.0)
        self.assertFalse(g.update(True, False, 9.9))

    def test_rule_resets_on_empty(self):
        rule = F.FusionRule()
        rule.calibrate(UPRIGHT)
        for i in range(70):
            alert, code = rule.step(i * 0.05, 0.0, -20.0, UPRIGHT)
        self.assertTrue(alert)
        self.assertEqual(rule.step(3.5, 0.0, -20.0, [0] * 8), (False, F.EMPTY))


class ReplayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xs = [F.inputs(r) for r in C.runs()]

    def test_vectorised_replay_matches(self):
        for x in self.xs:
            for p in (F.CURRENT, F.FUSION):
                alert, _ = F.replay(x, p)
                np.testing.assert_array_equal(alert, F.replay_v(x, p), err_msg=x.run.name)

    def test_current_matches_logged_alert(self):
        """The replayed firmware rule agrees with the chair's logged decision (0.1° rounding aside)."""
        for x in self.xs:
            logged = x.run.rows["alert"].to_numpy() == 1
            i0 = int(np.argmax(~logged))
            agree = np.mean(F.replay_v(x, F.CURRENT)[i0:] == logged[i0:])
            self.assertGreater(agree, 0.995, x.run.name)

    def test_fusion_beats_current(self):
        cur = F.objective(F.evaluate(self.xs, F.CURRENT))
        fus = F.objective(F.evaluate(self.xs, F.FUSION))
        self.assertGreater(fus, cur + 0.05)


if __name__ == "__main__":
    unittest.main()
