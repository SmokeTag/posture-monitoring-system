"""Tests for captures.py (loader) and fusion.py (the IMU + FSR alert rule).

    QT_QPA_PLATFORM=offscreen .venv/bin/python -m unittest discover -s analysis
"""

from __future__ import annotations

import json
import math
import subprocess
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

    def test_new_firmware_log(self):
        """reason column after c15 and the chair's own baseline in the event line."""
        text = synthetic().replace(",c15,tag", ",c15,reason,tag")
        text = text.replace("calid=6\n", "calid=6 fsr=1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0\n")
        lines = [l if l.startswith(("#", "t_ms")) else l.rsplit(",", 1)[0] + ",0," + l.rsplit(",", 1)[1]
                 for l in text.splitlines()]
        self.path.write_text("\n".join(lines) + "\n")
        run = C.load(self.path)
        self.assertIn("reason", run.rows)
        self.assertEqual(len(run.rows), 60)
        self.assertEqual(list(C.segments(run.rows)["tag"]), [1, 4])
        base = run.fsr_baseline(run.cals[1])
        self.assertEqual(base["seat_front_left"], 1.0)   # c8
        self.assertEqual(base["back_lower_left"], 7.0)   # c14

    def test_dropped_rows_untrust_the_window(self):
        """Rows the chair dropped (sdrop) inside the capture: no silent reconstruction."""
        lines = synthetic().splitlines()
        first = next(i for i, l in enumerate(lines) if l.startswith("1"))   # data rows
        del lines[first + 15:first + 20]                                   # 5 capture rows lost
        self.path.write_text("\n".join(lines) + "\n")
        with self.assertWarnsRegex(UserWarning, "calid=6 .*'# calibrating' not right before"):
            run = C.load(self.path)
        cal = run.cals[1]
        self.assertFalse(cal.trusted)
        x = F.inputs(run)
        self.assertTrue(np.isnan(x.base[cal.row:]).all())                  # not scored
        self.assertFalse(F.replay_v(x, F.FUSION).any())

    def test_seq_gap_untrusts_the_window(self):
        lines = synthetic().splitlines()
        i = next(i for i, l in enumerate(lines) if l.startswith("1")) + 16   # a capture row
        f = lines[i].split(",")
        f[10] = str(int(f[10]) + 3)                                         # 2 body samples lost
        lines[i] = ",".join(f)
        self.path.write_text("\n".join(lines) + "\n")
        with self.assertWarnsRegex(UserWarning, "calid=6 .*seq gap"):
            self.assertFalse(C.load(self.path).cals[1].trusted)

    def test_real_calibration_windows(self):
        """On every capture the window's IMU mean reproduces the logged reference."""
        for run in C.runs():
            for cal in run.cals:
                if not cal.in_file:
                    continue
                self.assertTrue(cal.trusted, f"{run.name} calid={cal.calid}")
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

    def test_capture_rejected_without_seat_rear(self):
        rule = F.FusionRule()
        self.assertFalse(rule.calibrate([2500, 2500, 0, 0, 2300, 2400, 2800, 2600]))   # perched
        self.assertEqual(rule.step(0, 30.0, 0.0, UPRIGHT), (False, F.INVALID))     # no decisions
        self.assertTrue(rule.calibrate(UPRIGHT))
        self.assertTrue(F.FusionRule(F.CURRENT).calibrate([0] * 8))                 # IMU-only rule

    def test_zero_baseline_falls_back_to_imu(self):
        """Defence in depth: the chair rejects such a capture, decide() still guards it."""
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
        g = F.Gate(3000)
        self.assertFalse(g.update(True, False, 0))
        self.assertFalse(g.update(True, False, 2950))
        self.assertTrue(g.update(True, False, 3000))
        self.assertTrue(g.update(False, False, 3100))    # between thresholds: held
        self.assertFalse(g.update(False, True, 3200))    # ok clears
        g.update(True, False, 4000)
        self.assertFalse(g.update(True, False, 6900))
        g.update(False, False, 7000)                     # a non-bad sample restarts the hold
        self.assertFalse(g.update(True, False, 9900))

    def test_rule_resets_on_empty(self):
        rule = F.FusionRule()
        rule.calibrate(UPRIGHT)
        for i in range(70):
            alert, code = rule.step(i * 50, 0.0, -20.0, UPRIGHT)
        self.assertTrue(alert)
        self.assertEqual(rule.step(3500, 0.0, -20.0, [0] * 8), (False, F.EMPTY))


def long_log(drop: bool, dp_bad: float = 30.0) -> str:
    """Uncalibrated lead-in, a capture (5 of its rows dropped if `drop`), then 6 s of
    tag 2 at dpitch = dp_bad and 6 s of tag 1 upright."""
    lines = ["# tag map 0=transition 1=ereto 2=curvado_frente", FSR_MAP, COLS]
    t, rows = 1000, []
    for i in range(10):
        rows.append(row(t, 5, 0.0, 0.0, UPRIGHT, 0, cal=0)); t += 50
    lines += rows
    lines.append("# calibrating (hold upright 1 s), from serial")
    cap = []
    for i in range(20):
        if i == 19:
            cap.append("# calibrated upright: pitch=0.0 roll=0.0 calid=6")
        cap.append(row(t, 6 if i == 19 else 5, 0.0, 0.0, UPRIGHT, 0, cal=int(i == 19))); t += 50
    if drop:
        del cap[3:8]
    lines += cap
    for tag, dp, n in ((0, 0.0, 10), (2, dp_bad, 120), (1, 0.0, 120)):
        for i in range(n):
            lines.append(row(t, 6, dp, 0.0, UPRIGHT, tag)); t += 50
    return "\n".join(lines) + "\n"


class EvaluationValidityTest(unittest.TestCase):
    """Rows under an untrusted calibration take no part in scoring or selection."""

    def load(self, name: str, text: str, warn: bool = False):
        path = Path(self.dir.name) / name
        path.write_text(text)
        if warn:
            with self.assertWarns(UserWarning):
                return F.inputs(C.load(path))
        return F.inputs(C.load(path))

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.dir.cleanup()

    def test_untrusted_window_not_scored(self):
        good = self.load("2026-10-21_Good-One_run1.csv", long_log(False))
        # a conflicting subject: its "bad" posture is tiny, would count as a miss
        bad = self.load("2026-10-21_Bad-One_run1.csv", long_log(True, dp_bad=1.0), warn=True)
        self.assertFalse(bad.valid.any())
        self.assertTrue(F.evaluate([bad], F.FUSION).empty)
        both = F.evaluate([good, bad], F.FUSION)
        self.assertEqual(set(both["subject"]), {"Good-One"})
        self.assertEqual(list(both["tag"]), [2, 1])
        # parameter selection sees exactly the trusted run
        ps = [F.FUSION, F.CURRENT, F.replace(F.FUSION, alert_deg=40.0, clear_deg=37.0)]
        pd_ = __import__("pandas").testing
        pd_.assert_series_equal(F.scores(F.search([good, bad], ps)), F.scores(F.search([good], ps)))
        # the same postures under a trusted window are scored
        self.assertEqual(len(F.evaluate([self.load("2026-10-21_Bad-Two_run1.csv",
                                                   long_log(False, dp_bad=1.0))], F.FUSION)), 2)

    def test_rejected_baseline_not_scored(self):
        """A baseline the chair would reject (seat rear < base_min) is not scored, for
        either rule: FUSION would be forced quiet there, CURRENT would not."""
        import pandas as pd
        r = pd.DataFrame({"cal": [1, 1, 1, 0]})
        base = np.full((4, 8), 2000.0)
        base[1, [F.SRL, F.SRR]] = 400.0                     # perched: 800 < base_min
        base[2] = np.nan                                     # untrusted window
        self.assertEqual(list(F.scored_rows(r, base)), [True, False, False, False])

    def test_rejected_capture_freezes_the_gate(self):
        """'# calibrating' .. '# calibrate rejected': no decisions, reference unchanged."""
        lines = long_log(False).splitlines()
        i = next(k for k, l in enumerate(lines) if l[0].isdigit() and l.endswith(",2")) + 30
        n_capture = 20
        lines.insert(i, "# calibrating (hold upright 1 s), from button")
        lines.insert(i + n_capture, "# calibrate rejected: seat rear unloaded "
                                    "(sit back fully and recalibrate) rear=0")
        x = self.load("2026-10-21_Rej-One_run1.csv", "\n".join(lines) + "\n")
        cap = x.run.captures[-1]
        self.assertEqual(cap.outcome, "rejected")
        self.assertEqual(cap.end - cap.start, n_capture - 1)
        self.assertTrue(x.frozen[cap.start:cap.end].all())
        self.assertFalse(x.frozen[cap.end])
        self.assertFalse(x.recal[cap.start:cap.end + 1].any())
        self.assertEqual(len(x.run.cals), 1)
        alert, code = F.replay(x, F.FUSION)
        np.testing.assert_array_equal(alert, F.replay_v(x, F.FUSION))
        # frozen: the alert held through the capture is the one before it
        self.assertTrue((alert[cap.start:cap.end] == alert[cap.start - 1]).all())


class FirmwareReplayTest(unittest.TestCase):
    """The real chair.ino's output (test_chair's fusion scenario: rejected captures,
    slump, recline, lean, empty seat, clear) replayed by the Python rule."""

    @classmethod
    def setUpClass(cls):
        tests = C.ROOT / "firmware" / "test"
        subprocess.run(["make", "-s", "-C", str(tests), "test_chair"], check=True, capture_output=True)
        cls.dir = tempfile.TemporaryDirectory()
        out = Path(cls.dir.name) / "fusion.out"
        subprocess.run([str(tests / "test_chair"), str(out)], check=True, capture_output=True)
        m = json.loads((Path(F.__file__).with_name("chair_map.json")).read_text())
        fsr_map = "# fsr map " + " ".join(f"c{ch}={spot}" for spot, ch in m.items())
        log = Path(cls.dir.name) / "2026-10-22_Firmware-Test_run1.csv"
        log.write_text(fsr_map + "\n" + out.read_text())
        cls.x = F.inputs(C.load(log))

    @classmethod
    def tearDownClass(cls):
        cls.dir.cleanup()

    def test_alert_and_reason_match(self):
        x, rows = self.x, self.x.run.rows
        self.assertEqual([c.outcome for c in x.run.captures], ["rejected", "calibrated", "rejected"])
        alert, code = F.replay(x, F.FUSION)
        np.testing.assert_array_equal(alert, rows["alert"].to_numpy() == 1)
        logged = rows["reason"].to_numpy()
        decided = logged >= 0
        self.assertGreater(decided.sum(), 300)
        np.testing.assert_array_equal(code[decided], logged[decided])
        self.assertEqual(set(logged[decided]), {F.OK, F.IMU_DEV, F.SLUMP, F.LEAN, F.RECLINE_OK, F.EMPTY})


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

    def test_cpp_fixture_up_to_date(self):
        """firmware/test/fixtures/fusion_cases.txt = this rule's output (regenerate: --fixture)."""
        self.assertEqual(F.FIXTURE.read_text(), "\n".join(F.fixture_lines(self.xs)) + "\n")

    def test_fusion_beats_current(self):
        cur = F.objective(F.evaluate(self.xs, F.CURRENT))
        fus = F.objective(F.evaluate(self.xs, F.FUSION))
        self.assertGreater(fus, cur + 0.05)


if __name__ == "__main__":
    unittest.main()
