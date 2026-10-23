"""ble_chair.py: beacon decoding (Beacon.h layout) and the chair CSV it rebuilds."""

import math
import struct
import unittest

import ble_chair as B
import live_posture as LP


def payload(seq=0, t16=0, flags=0, bflags=0, calid=7, reason=-1, imu=-1, dp=B.NAN16, dr=B.NAN16,
            vbat=0xFFFF, fsr=(0,) * 8):
    b = struct.pack("<BBHBBBbbhhH", B.BEACON_VER, seq, t16, flags, bflags, calid, reason, imu, dp, dr, vbat)
    for i in range(4):
        b += (fsr[2 * i] | fsr[2 * i + 1] << 12).to_bytes(3, "little")
    return b


class Decode(unittest.TestCase):
    def test_fields(self):
        b = B.decode(payload(5, 0x1234, B.BK_CAL | B.BK_BODY, 1 | B.BB_MUTE, 9, 0, 0, 43, -25, 3987,
                             (2100, 2200, 2300, 2400, 1234, 0, 4095, 1)))
        self.assertEqual((b["seq"], b["t16"], b["calid"], b["reason"], b["imu"], b["vbat"]),
                         (5, 0x1234, 9, 0, 0, 3987))
        self.assertAlmostEqual(b["dpitch"], 4.3)
        self.assertAlmostEqual(b["droll"], -2.5)
        self.assertEqual(b["fsr"], [2100, 2200, 2300, 2400, 1234, 0, 4095, 1])

    def test_not_ours(self):
        self.assertIsNone(B.decode(payload()[:-1]))
        self.assertIsNone(B.decode(b"\x02" + payload()[1:]))
        b = B.decode(payload())
        self.assertTrue(math.isnan(b["dpitch"]) and math.isnan(b["droll"]))


class Lines(unittest.TestCase):
    def test_rows_parse_as_chair_csv(self):
        c = B.Beacons()
        lines = c.lines(B.decode(payload(250, 0xFFF0)), 100.0)
        self.assertEqual(lines[:3], ["# unit=raw", "# proto=ble1", B.HEADER])
        fields, tail = LP.header_fields(B.HEADER), LP.tail_fields(B.HEADER)
        row, fsr = LP.parse_row(lines[-1], fields, tail)
        self.assertEqual(row["seq"], -1)                 # body silent
        self.assertEqual(len(fsr), 8)
        # body up, calibrated: t unwraps across 0xFFFF, seq counts rows, events on the edges
        lines = c.lines(B.decode(payload(252, 0x0010, B.BK_CAL | B.BK_BODY, 1, 8, 0, 0, 30, -40, 3900)),
                        100.1)
        self.assertIn("# body link up (ble)", lines)
        self.assertIn("# calibrated upright: calid=8 (ble)", lines)
        row, _ = LP.parse_row(lines[-1], fields, tail)
        self.assertEqual(row["t_ms"], 0xFFF0 + 0x20)
        self.assertEqual((row["seq"], row["cal"], row["bctl"], row["vbat"], row["imu"]), (1, 1, 1, 3900, 0))
        self.assertAlmostEqual(row["dev"], 5.0)
        self.assertTrue(LP.imu_valid(row))
        self.assertEqual(c.stats(-60), "# ble beacons=2 lost=1 rssi=-60")
        self.assertEqual(c.lines(B.decode(payload(252, 0x0010)), 100.11), [])   # same beacon again
        lines = c.lines(B.decode(payload(253, 0x0080, B.BK_BODY, 1, 9, -1, 0, 30, -40, 3900)), 100.2)
        self.assertIn("# calibration cleared calid=9", lines)


if __name__ == "__main__":
    unittest.main()
