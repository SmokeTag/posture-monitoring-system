"""USB/BLE failover through the real CSV parsers, without hardware or windows."""

import unittest
from unittest.mock import Mock, patch

import auto_chair as A
import ble_chair as B
import mostra as X


class Failover(unittest.TestCase):
    def setUp(self):
        self.clock = patch.object(A.time, "monotonic", return_value=1000.0)
        self.wall = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.reader = A.AutoChairReader("/dev/chair", trail_n=200, address="AA:BB:CC:DD:EE:FF")
        self.reader.usb._handle("# unit=raw")
        self.reader.usb._handle(B.HEADER)

    def usb(self, t, wall, **changes):
        self.wall.return_value = wall
        values = dict(t_ms=t, cal=1, calid=7, dev=2, alert=0, dpitch=2, droll=0,
                      pitch=3, roll=4, seq=int(t / 50), imu=0, reason=0,
                      **{f"c{c}": 1000 for c in range(8, 16)})
        values.update(changes)
        self.reader.usb._handle(",".join(str(values.get(k, 0)) for k in B.HEADER.split(",")))
        self.reader.usb.connected = True

    def ble(self, t, wall, *, body=True, alert=False, cal=True):
        self.wall.return_value = wall
        b = dict(seq=int(t / 50) & 0xFF, t16=t & 0xFFFF,
                 flags=(B.BK_CAL if cal else 0) | (B.BK_BODY if body else 0) | (B.BK_ALERT if alert else 0),
                 bflags=1, calid=7, reason=1 if alert else 0, imu=0,
                 dpitch=2, droll=0, vbat=4000, fsr=[1000] * 8)
        self.reader.ble._beacon(b, -60, wall)

    def poll(self, now):
        self.reader._poll(now)
        return self.reader.snapshot()[0]

    def test_usb_wins_and_inactive_history_is_never_replayed(self):
        self.ble(200000, 1000.0)
        self.usb(200000, 1000.01)
        self.assertEqual(len(self.poll(1000.02)), 1)
        for i in range(1, 4):
            self.ble(200000 + 50 * i, 1000 + 0.05 * i)
        self.usb(200200, 1000.20)
        rows = self.poll(1000.21)
        self.assertEqual([r["_transport"] for r in rows], ["USB", "USB"])
        self.reader.usb.connected = False
        self.poll(1000.22)
        self.assertFalse(self.reader.connected)          # BLE cache is older
        self.ble(200250, 1000.25)
        rows = self.poll(1000.26)
        self.assertEqual([r["_transport"] for r in rows], ["USB", "USB", "BLE"])

    def test_fallback_and_return_keep_clock_visit_and_alert_count(self):
        self.usb(0x2FFF0, 1000.0, alert=1, reason=1)
        self.poll(1000.01)
        self.reader.usb.connected = False
        self.ble(0x30022, 1000.05, alert=True)         # low 16 bits wrapped
        self.poll(1000.06)
        self.usb(0x30054, 1000.10, alert=1, reason=1)
        rows = self.poll(1000.11)
        self.assertEqual([r["_transport"] for r in rows], ["USB", "BLE", "USB"])
        self.assertEqual([round(r["t_ms"]) for r in rows], [0, 50, 100])
        ex = X.Exhibit(X.DayStats("2026-10-24"))
        for r in rows:
            ex.feed(r, seated=True)
        self.assertAlmostEqual(ex.visit.seated_s, 0.1)
        self.assertEqual(ex.visit.alerts, 1)
        self.assertEqual(ex.day.visitors, 0)

    def test_same_chair_sample_on_both_transports_is_counted_once(self):
        self.usb(100000, 1000.0)
        self.poll(1000.01)
        self.reader.usb.connected = False
        self.ble(100001, 1000.02)                    # same state, beacon sent 1 ms after USB row
        self.assertEqual(len(self.poll(1000.03)), 1)
        self.ble(100050, 1000.05)
        self.assertEqual(len(self.poll(1000.06)), 2)

    def test_silent_usb_falls_back_even_if_port_is_still_open(self):
        self.usb(100000, 1000.0)
        self.poll(1000.01)
        self.ble(101050, 1001.05)
        self.poll(1001.06)
        self.assertEqual(self.reader.transport, "BLE")
        self.assertTrue(self.reader.connected)
        self.usb(101100, 1001.10)
        self.poll(1001.11)
        self.assertEqual(self.reader.transport, "USB")

    def test_open_usb_without_data_does_not_block_ble_and_both_can_go_offline(self):
        self.reader.usb.connected = True
        self.reader.usb._handle("# rx pkts=20 lost=0")
        self.reader.usb._handle("1,not,a,valid,row")
        self.ble(100000, 1000.0)
        self.poll(1000.01)
        self.assertEqual(self.reader.transport, "BLE")
        self.poll(1002.01)
        self.assertFalse(self.reader.connected)
        self.assertIsNone(self.reader.transport)
        self.assertFalse(self.reader.body_up)
        self.ble(105000, 1005.0)
        rows = self.poll(1005.01)
        self.assertTrue(self.reader.connected)
        self.assertEqual([round(r["t_ms"]) for r in rows], [0, 5000])

    def test_body_silent_rows_keep_the_chair_and_fsr_connection_alive(self):
        self.ble(100000, 1000.0, body=False)
        self.assertEqual(self.poll(1000.01), [])
        self.assertTrue(self.reader.connected)
        self.assertFalse(self.reader.body_up)
        self.assertEqual(self.reader.raw_fsr, dict.fromkeys(range(8, 16), 1000))

    def test_commands_use_active_usb_and_are_never_queued(self):
        self.reader.usb.send = Mock()
        self.reader.send("c")
        self.usb(100000, 1000.0)
        self.poll(1000.01)
        self.reader.send("c")
        self.reader.usb.send.assert_called_once_with("c")
        self.reader.usb.connected = False
        self.ble(100050, 1000.05)
        self.poll(1000.06)
        self.reader.send("r")
        self.usb(100100, 1000.10)
        self.poll(1000.11)
        self.reader.usb.send.assert_called_once_with("c")

    def test_bluetooth_failure_does_not_interrupt_usb(self):
        self.reader.ble.error = "Bluetooth adapter unavailable"
        self.usb(100000, 1000.0)
        self.poll(1000.01)
        self.assertEqual(self.reader.transport, "USB")
        self.assertTrue(self.reader.connected)
        self.assertEqual(self.reader.ble.address, "AA:BB:CC:DD:EE:FF")

    def test_reconnect_discards_old_usb_samples_until_new_data(self):
        self.usb(100000, 1000.0)
        self.poll(1000.01)
        with patch.object(A.LP.ChairReader, "_open_port", return_value=Mock()):
            self.reader.usb._open_port()
        self.ble(100050, 1000.05)
        self.poll(1000.06)
        self.assertEqual(self.reader.transport, "BLE")
        self.usb(100100, 1000.10)
        self.poll(1000.11)
        self.assertEqual(self.reader.transport, "USB")

    def test_switch_does_not_replay_an_older_calibration_clear(self):
        self.usb(100000, 1000.0)
        self.reader.usb._handle("# calibrated upright: calid=7")
        self.poll(1000.01)
        event = self.reader.cal_event
        self.reader.usb.connected = False
        self.ble(100050, 1000.05)
        self.reader.ble.cal_event = ("calibration cleared calid=6", 999.0)
        self.poll(1000.06)
        self.assertEqual(self.reader.cal_event, event)

    def test_new_clear_is_forwarded_even_while_body_is_silent(self):
        self.usb(100000, 1000.0)
        self.poll(1000.01)
        self.ble(100050, 1000.05)
        self.reader.usb.connected = False
        self.ble(100100, 1000.10, body=False, cal=False)
        self.poll(1000.11)
        self.assertTrue(self.reader.cal_event[0].startswith("calibration cleared"))
        self.assertFalse(self.reader.body_up)


if __name__ == "__main__":
    unittest.main()
