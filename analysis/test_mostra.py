"""mostra.py's visitor logic: categories, posture names, visits and day numbers.

    QT_QPA_PLATFORM=offscreen .venv/bin/python -m unittest discover -s analysis
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import ble_chair as B
import mostra as X


def row(t_s: float, **kw) -> dict:
    r = dict(t_ms=t_s * 1000, cal=1, calid=5, dev=2.0, alert=0, dpitch=1.0, droll=1.0,
             imu=0, reason=X.R_OK)
    r.update(kw)
    return r


def sit(ex: X.Exhibit, t0: float, t1: float, **kw):
    """Rows every 50 ms from t0 to t1 (s)."""
    for i in range(int((t1 - t0) * 20)):
        ex.feed(row(t0 + i * 0.05, **kw), seated=True)


class Category(unittest.TestCase):
    def test_states(self):
        self.assertEqual(X.category(row(0)), X.GOOD)
        self.assertEqual(X.category(row(0, reason=X.R_RECLINE_OK)), X.GOOD)
        self.assertEqual(X.category(row(0, reason=X.R_SLUMP)), X.WARN)
        self.assertEqual(X.category(row(0, reason=X.R_IMU_DEV, alert=1)), X.ALERT)
        self.assertEqual(X.category(row(0, reason=X.R_EMPTY)), X.NONE)
        self.assertEqual(X.category(row(0, cal=0, reason=-1)), X.NONE)
        self.assertEqual(X.category(row(0, imu=2)), X.NONE)

    def test_firmware_without_reason(self):
        r = row(0, dev=12.0)
        del r["reason"]
        self.assertEqual(X.category(r), X.WARN)


class PostureName(unittest.TestCase):
    def test_names_follow_the_collar_axes(self):
        """droll = forward (−) / back (+), dpitch = left (−) / right (+)."""
        self.assertEqual(X.posture_name(row(0, reason=X.R_IMU_DEV, droll=-20, dpitch=2)),
                         "curvado para a frente")
        self.assertEqual(X.posture_name(row(0, reason=X.R_IMU_DEV, droll=15, dpitch=2)),
                         "inclinado para trás")
        self.assertEqual(X.posture_name(row(0, reason=X.R_IMU_DEV, droll=1, dpitch=-14)),
                         "inclinado para a esquerda")
        self.assertEqual(X.posture_name(row(0, reason=X.R_LEAN, dpitch=8)),
                         "inclinado para a direita")
        self.assertEqual(X.posture_name(row(0, reason=X.R_SLUMP)), "escorregado no assento")
        self.assertEqual(X.posture_name(row(0)), "")


class Visits(unittest.TestCase):
    def setUp(self):
        self.saved = []
        self.ex = X.Exhibit(X.DayStats("2026-10-24"), on_day_change=self.saved.append)

    def test_no_visit_before_calibrating(self):
        sit(self.ex, 0, 5, cal=0, reason=-1)
        self.assertIsNone(self.ex.visit)

    def test_stats_of_a_visit(self):
        sit(self.ex, 0, 20)                                         # good
        sit(self.ex, 20, 25, reason=X.R_IMU_DEV)                    # attention…
        sit(self.ex, 25, 30, reason=X.R_IMU_DEV, alert=1)           # …then alert
        sit(self.ex, 30, 40)
        v = self.ex.visit
        self.assertAlmostEqual(v.seated_s, 40, delta=0.2)
        self.assertAlmostEqual(v.good_pct, 75, delta=1)
        self.assertEqual(v.alerts, 1)
        self.assertAlmostEqual(v.best_streak_s, 20, delta=0.2)

    def test_recalibrating_is_the_same_visitor(self):
        sit(self.ex, 0, 10)
        sit(self.ex, 10, 20, calid=6)
        self.assertAlmostEqual(self.ex.visit.seated_s, 20, delta=0.2)

    def test_chair_clearing_calibration_ends_the_visit(self):
        sit(self.ex, 0, 30)
        first = self.ex.visit
        self.ex.feed(row(40, cal=0, calid=6, reason=-1))   # empty interval missed by the laptop
        self.assertIsNone(self.ex.visit)
        self.assertIs(self.ex.last, first)
        self.assertEqual(self.ex.day.visitors, 1)
        self.assertEqual(self.saved, [self.ex.day])
        self.ex.feed(row(42, cal=0, calid=6, reason=-1))
        self.assertEqual(self.ex.day.visitors, 1)
        self.ex.feed(row(44, calid=7))
        self.assertIsNot(self.ex.visit, first)
        self.assertEqual(self.ex.visit.calid, 7)
        self.assertEqual(self.ex.visit.seated_s, 0)

    def test_leaving_ends_the_visit_into_the_day(self):
        sit(self.ex, 0, 30)
        self.assertFalse(self.ex.seat(False, 100.0))
        self.assertFalse(self.ex.seat(False, 100.0 + X.LEAVE_S - 1))
        self.assertTrue(self.ex.seat(False, 100.0 + X.LEAVE_S))
        self.assertIsNone(self.ex.visit)
        self.assertEqual(self.ex.day.visitors, 1)
        self.assertEqual(self.saved, [self.ex.day])
        self.assertAlmostEqual(self.ex.last.good_pct, 100)

    def test_sitting_back_down_keeps_the_visit(self):
        sit(self.ex, 0, 30)
        self.ex.seat(False, 100.0)
        self.ex.seat(True, 105.0)
        self.assertFalse(self.ex.seat(False, 100.0 + X.LEAVE_S + 1))
        self.assertIsNotNone(self.ex.visit)

    def test_short_visits_dont_count(self):
        sit(self.ex, 0, X.MIN_VISIT_S - 5)
        self.ex.end()
        self.assertEqual(self.ex.day.visitors, 0)

    def test_old_calibration_doesnt_restart_a_visit(self):
        """With --keep-cal the chair stays calibrated: only a new calid starts one."""
        sit(self.ex, 0, 30)
        self.ex.end()
        sit(self.ex, 30, 40)
        self.assertIsNone(self.ex.visit)
        sit(self.ex, 40, 50, calid=7)
        self.assertIsNotNone(self.ex.visit)

    def test_empty_seat_isnt_counted(self):
        sit(self.ex, 0, 10)
        for i in range(100):
            self.ex.feed(row(10 + i * 0.05), seated=False)
        self.assertAlmostEqual(self.ex.visit.seated_s, 10, delta=0.2)

    def test_leaving_breaks_the_streak(self):
        sit(self.ex, 0, 20)
        for i in range(40):                                         # 2 s away, visit kept
            self.ex.feed(row(20 + i * 0.05), seated=False)
        sit(self.ex, 22, 42)
        sit(self.ex, 42, 44, imu=2)                                 # invalid IMU breaks it too
        sit(self.ex, 44, 50)
        self.assertAlmostEqual(self.ex.visit.best_streak_s, 20, delta=0.2)

    def test_time_going_back_resets_the_timeline(self):
        sit(self.ex, 1000, 1010)
        sit(self.ex, 0, 10)                                         # chair rebooted
        self.assertTrue(all(t < 10 for t, _ in self.ex.timeline))
        self.assertLessEqual(len(self.ex.timeline), 200)

    def test_sparse_ble_rows(self):
        ex = X.Exhibit(X.DayStats("2026-10-24"), max_dt=2.0)
        for t in (0, 0.5, 2, 4):
            ex.feed(row(t))
        self.assertAlmostEqual(ex.visit.seated_s, 4)
        self.assertAlmostEqual(ex.visit.good_s, 4)
        self.assertTrue(all(ex.strip()[-17:] == X.GOOD))
        ex.feed(row(10))
        self.assertAlmostEqual(ex.visit.seated_s, 6)
        self.assertEqual(ex.strip()[-10], X.NONE)

    def test_usb_sparse_rows_still_clamped(self):
        self.ex.feed(row(0))
        self.ex.feed(row(2))
        self.assertAlmostEqual(self.ex.visit.seated_s, 0.25)
        self.assertEqual(self.ex.strip()[-3], X.NONE)

    def test_timeline(self):
        sit(self.ex, 0, 10)
        sit(self.ex, 10, 12, reason=X.R_IMU_DEV, alert=1)
        s = self.ex.strip()
        self.assertEqual(s[-1], X.ALERT)
        self.assertEqual(s[-20], X.GOOD)
        self.assertEqual(s[0], X.NONE)                              # before anything was fed


class Day(unittest.TestCase):
    def test_round_trip_and_numbers(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = X.DayStats("2026-10-24")
            d.add(X.Visit(1, seated_s=60, good_s=45, alerts=2))
            d.add(X.Visit(2, seated_s=60, good_s=57, alerts=0))
            d.save(Path(tmp))
            e = X.DayStats.load(Path(tmp), "2026-10-24")
            self.assertEqual((e.visitors, e.alerts), (2, 2))
            self.assertAlmostEqual(e.mean_pct, 85)
            self.assertAlmostEqual(e.best_pct, 95)
            self.assertEqual(X.DayStats.load(Path(tmp), "2026-10-25").visitors, 0)


class BleVisitorFlow(unittest.TestCase):
    """Drive the real animation callback and beacon parser without hardware or a GUI loop."""

    @classmethod
    def setUpClass(cls):
        X.plt.switch_backend("Agg")

    def test_sparse_empty_beacons_end_visit_before_the_next_calibration(self):
        self.run_flow(body_up=True)

    def test_chair_clear_ends_visit_even_without_body_rows_or_fresh_fsr(self):
        self.run_flow(body_up=False)

    def run_flow(self, body_up):
        reader = B.BleChairReader(None, trail_n=200)
        clock = [100.0]
        seq = [0]

        def drive():
            update = animation.call_args.args[1]

            def frame(t, *, beacon=False, cal=True, calid=5, occupied=True, body=True):
                clock[0] = 100.0 + t
                if beacon:
                    seq[0] += 1
                    flags = (B.BK_CAL if cal else 0) | (B.BK_BODY if body else 0)
                    reader._beacon(dict(seq=seq[0], t16=int(t * 1000), flags=flags,
                                        bflags=0, calid=calid, reason=X.R_OK if occupied else X.R_EMPTY,
                                        imu=0, dpitch=1.0, droll=1.0, vbat=3900,
                                        fsr=[1000 if occupied else 0] * 8), -60, clock[0])
                update(None)

            for i in range(21):
                frame(i * 1.5, beacon=True)
            # Run every 50 ms: the 1.0 s staleness bug only appears between beacons.
            for i in range(240):
                if body_up:
                    frame(31.5 + i * 0.05, beacon=i % 30 == 0, occupied=False)
                else:
                    # A single authoritative clear must work even if FSR freshness is unknown.
                    frame(31.5 + i * 0.05, beacon=i == 0, occupied=False,
                          cal=False, calid=6, body=False)

            d = X.DayStats.load(folder, X.datetime.date.today().isoformat())
            self.assertEqual(d.visitors, 1)
            self.assertEqual(d.good_pcts, [100.0])
            # Clear/recalibrate for the next person; the former visit must stay saved once.
            frame(44, beacon=True, cal=False, calid=6, occupied=False)
            for i in range(15):
                frame(45 + i * 1.5, beacon=True, calid=7)
            frame(67, beacon=True, cal=False, calid=8, occupied=False)
            d = X.DayStats.load(folder, X.datetime.date.today().isoformat())
            self.assertEqual(d.visitors, 2)
            self.assertEqual(d.good_pcts, [100.0, 100.0])

        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            with (patch("sys.argv", ["mostra.py", "--ble", "--window", "--no-ref",
                                     "--stats-dir", tmp]),
                  patch.object(B, "BleChairReader", return_value=reader),
                  patch.object(reader, "start"),
                  patch.object(X.time, "monotonic", side_effect=lambda: clock[0]),
                  patch.object(X, "FuncAnimation") as animation,
                  patch.object(X.plt, "show", side_effect=drive)):
                try:
                    self.assertEqual(X.main(), 0)
                finally:
                    X.plt.close("all")


class Seat(unittest.TestCase):
    def test_seat_sum_uses_the_seat_spots(self):
        fmap = {"seat_front_left": 8, "seat_front_right": 9, "seat_rear_left": 10,
                "seat_rear_right": 11, "back_lower_left": 12}
        raw = {8: 100, 9: 200, 10: 300, 11: 400, 12: 5000}
        self.assertEqual(X.seat_raw(raw, fmap), 1000)
        self.assertIsNone(X.seat_raw(raw, {"seat_front_left": 8}))
        self.assertIsNone(X.seat_raw({}, fmap))


if __name__ == "__main__":
    unittest.main()
