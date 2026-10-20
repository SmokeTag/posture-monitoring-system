"""Tests for live_posture.py's CSV parsing and body-status logic (no board,
no window). The main test feeds it the real chair firmware's output: the
two-board system test (firmware/test/test_system: chair.ino + body.ino on host
stubs) is built and run, and its chair serial stream is parsed line by line.

    QT_QPA_PLATFORM=offscreen .venv/bin/python -m unittest discover -s analysis
"""

from __future__ import annotations

import subprocess
import tempfile
import unittest
from pathlib import Path

import live_posture as LP

ROOT = Path(__file__).resolve().parent.parent
TESTS = ROOT / "firmware" / "test"

# firmware v1 (before the body's report): column line + rows, as captured
V1_HEADER = "t_ms,cal,dev,alert,led,dpitch,droll,pitch,roll,seq,c8,c9,c10,c11,c12,c13,c14,c15"
V1_ROWS = [
    "1000,1,3.2,0,0,1.0,-3.0,5.0,2.0,41,-1,-1,5400,-1,-1,-1,-1,-1",
    "1050,1,12.5,1,1,12.0,-3.5,16.0,1.5,42,-1,-1,5410,-1,-1,-1,-1,-1",
    "1100,0,0.0,0,0,0.0,0.0,0.0,0.0,-1,-1,-1,-1,-1,-1,-1,-1,-1",
]
V2_FIELDS = ("t_ms,cal,calid,dev,alert,dpitch,droll,pitch,roll,boot,seq,"
             "bctl,bcal,bcalid,balert,bmotor,btest,bseq,bage").split(",")


def reader() -> LP.ChairReader:
    return LP.ChairReader("/dev/null", None, trail_n=60)


def feed(r: LP.ChairReader, lines):
    for line in lines:
        r._handle(line)


def row(seq, *, boot=0x5a, cal=1, calid=7, alert=0, bctl=1, bcal=1, bcalid=7, balert=0,
        bmotor=0, btest=0, bseq=None, bage=2):
    """A v2 row dict; bseq defaults to seq - bage."""
    if bseq is None:
        bseq = (seq - bage) & 0xFFFF if bage >= 0 else -1
    return dict(t_ms=0, cal=cal, calid=calid, dev=0, alert=alert, dpitch=0, droll=0, pitch=0,
                roll=0, boot=boot, seq=seq, bctl=bctl, bcal=bcal, bcalid=bcalid, balert=balert,
                bmotor=bmotor, btest=btest, bseq=bseq, bage=bage)


class RealFirmwareStream(unittest.TestCase):
    """The chair stream of firmware/test/test_system, through ChairReader."""

    @classmethod
    def setUpClass(cls):
        subprocess.run(["make", "-s", "-C", str(TESTS), "test_system"], check=True,
                       capture_output=True)
        with tempfile.TemporaryDirectory() as d:
            chair, body = Path(d) / "chair.out", Path(d) / "body.out"
            subprocess.run([str(TESTS / "test_system"), str(chair), str(body)], check=True,
                           capture_output=True)
            cls.lines = chair.read_text().splitlines()

    def test_header_drives_columns(self):
        r = reader()
        feed(r, self.lines)
        self.assertEqual(r.fields, V2_FIELDS)
        self.assertTrue(r.rows)
        self.assertEqual(len(r.fsr[-1][1]), 8)          # c8..c15 found after the new columns

    def test_every_body_report_confirms(self):
        """Replays the stream; at every row the status is computed as the
        live view would. The system is consistent, so: never "≠"; ✓ in steady
        chair control; fallback and unknown bases never count as a mismatch."""
        r = reader()
        seen = {"✓ applied": 0, "unconfirmed": 0, "FALLBACK": 0, "≠ applied": 0}
        for line in self.lines:
            n0 = len(r.rows)
            last = r.rows[-1] if r.rows else None
            r._handle(line)
            if not r.rows or (len(r.rows) == n0 and r.rows[-1] is last):
                continue
            text = " ".join(LP.body_lines(list(r.rows))[:2])
            for k in seen:
                seen[k] += k in text
        self.assertEqual(seen["≠ applied"], 0)
        self.assertGreater(seen["✓ applied"], 200)
        self.assertGreater(seen["FALLBACK"], 50)
        self.assertGreater(seen["unconfirmed"], 0)       # e.g. the chair's no-basis ACK

    def test_motor_test_is_not_an_alert(self):
        r = reader()
        tested = False
        for line in self.lines:
            r._handle(line)
            if r.rows and r.rows[-1].get("btest") == 1:
                self.assertEqual(LP.body_lines(list(r.rows))[2], "motor ● ON test")
                self.assertEqual(r.rows[-1]["balert"], 0)
                tested = True
        self.assertTrue(tested)


class OldFirmware(unittest.TestCase):
    def test_v1_with_header(self):
        r = reader()
        feed(r, [V1_HEADER] + V1_ROWS)
        self.assertEqual(r.fields, LP.FIELDS)
        self.assertEqual([x["seq"] for x in r.rows], [41, 42])
        self.assertFalse(r.body_up)                     # last row: seq -1
        self.assertEqual(LP.body_lines(list(r.rows)),
                         ["body  (old firmware)", "      no report", "motor ● ON"])
        self.assertEqual(r.fsr[-2][1][10], LP.M.ohms_to_us(5410))

    def test_v1_without_header(self):
        """No header seen (e.g. a log started mid-stream): v1 layout assumed."""
        r = reader()
        feed(r, V1_ROWS[:2])
        self.assertEqual(r.rows[-1]["led"], 1)

    def test_header_change_drops_old_rows(self):
        r = reader()
        feed(r, [V1_HEADER] + V1_ROWS[:2])
        feed(r, [",".join(V2_FIELDS + [f"c{c}" for c in LP.M.CHANNELS])])
        self.assertEqual(len(r.rows), 0)


class Confirmation(unittest.TestCase):
    """When body_lines() may say ✓ / ≠, and when only "unconfirmed"."""

    def stream(self, n=10, **kw):
        return [row(s, **kw) for s in range(100, 100 + n)]

    def check(self, rows):
        return LP.body_lines(rows)[1].strip()

    def test_steady_state(self):
        self.assertEqual(self.check(self.stream()), "✓ applied")
        self.assertEqual(LP.body_lines(self.stream())[0], "body  chair, age 2")

    def test_mismatch(self):
        rows = self.stream()
        rows[-3]["alert"] = 1                           # decided alert, body reports none
        self.assertEqual(self.check(rows), "≠ applied")
        rows = self.stream()
        rows[-3]["calid"] = 8                           # another reference
        self.assertEqual(self.check(rows), "≠ applied")

    def test_unknown_basis_or_saturated_age(self):
        for kw in (dict(bage=-1), dict(bage=254, bseq=5)):
            rows = self.stream()
            rows[-1] = row(109, **kw)
            self.assertEqual(self.check(rows), "unconfirmed")
        self.assertEqual(LP.body_lines(self.stream()[:-1] + [row(109, bage=254, bseq=5)])[0],
                         "body  chair, age 254+")

    def test_decision_not_in_buffer(self):
        self.assertEqual(self.check([row(100)]), "unconfirmed")

    def test_other_session_never_matches(self):
        rows = self.stream()
        for x in rows[:-1]:
            x["boot"] = 0x11                            # same seqs, other body boot
            x["alert"] = 1
        self.assertEqual(self.check(rows), "unconfirmed")

    def test_seq_going_back_stops_the_search(self):
        rows = [row(s) for s in (105, 106, 107)] + [row(s) for s in (100, 101, 102)]
        rows[1]["alert"] = 1                            # seq 106 "decided" an alert, earlier
        rows[-1] = row(108, bage=2)                     # bseq 106, but seq went back before it
        self.assertEqual(self.check(rows), "unconfirmed")

    def test_seq_wrap(self):
        rows = [row(s & 0xFFFF) for s in range(65530, 65538)]
        self.assertEqual(rows[-1]["bseq"], 65535)
        self.assertEqual(self.check(rows), "✓ applied")

    def test_fallback_is_not_a_mismatch(self):
        rows = self.stream(bctl=2, balert=1, bage=60)   # its own alert, chair says none
        self.assertEqual(LP.body_lines(rows)[:2], ["body  FALLBACK (alone)", "      ref = chair's"])
        rows[-1]["bcalid"] = 9
        self.assertEqual(self.check(rows), "ref ≠ chair's")

    def test_none_yet(self):
        rows = [row(0, bctl=0, bcal=0, bcalid=0, bage=-1)]
        self.assertEqual(LP.body_lines(rows)[0], "body  no chair decision")


class Parsing(unittest.TestCase):
    def test_raw_fsr_conversion_and_legacy_units(self):
        r = reader()
        header = ",".join(V2_FIELDS + [f"c{c}" for c in LP.M.CHANNELS])
        sample = ",".join(str(v) for v in row(100).values())
        feed(r, ["# unit=raw", header, sample + ",0,7,8,1365,2048,4094,0,0"])
        loads = r.fsr[-1][1]
        self.assertEqual(loads[8], 0)
        self.assertEqual(loads[9], 0)                  # below the open-FSR threshold
        self.assertGreater(loads[10], 0)
        self.assertAlmostEqual(loads[11], 1e6 / 6600) # raw 1365 = 1/3 full scale
        self.assertGreater(loads[12], loads[11])
        self.assertGreater(loads[13], loads[12])
        feed(r, ["# unit=ohms (-1 = open)", header, sample + ",6600,-1,-1,-1,-1,-1,-1,-1"])
        self.assertAlmostEqual(r.fsr[-1][1][8], loads[11])

    def test_header_fields(self):
        cols = V2_FIELDS + [f"c{c}" for c in LP.M.CHANNELS]
        self.assertEqual(LP.header_fields(",".join(cols)), V2_FIELDS)

    def test_parse_row_rejects_junk(self):
        self.assertIsNone(LP.parse_row("# rx pkts=1", V2_FIELDS))
        self.assertIsNone(LP.parse_row("1,2,3", V2_FIELDS))
        self.assertIsNone(LP.parse_row("12,a" + ",0" * 30, V2_FIELDS))

    def test_rx_summary_shows_bad(self):
        s = LP.rx_summary("rx pkts=0 lost=0 dup=0 old=0 rst=0 bad=20 sdrop=0 edrop=0 dt=1000")
        self.assertEqual(s, "rx pkts=0 lost=0 bad=20")


if __name__ == "__main__":
    unittest.main()
