"""Wireless reader of the chair unit: its BLE beacon, as the chair's CSV.

The chair (firmware/chair) advertises its live state ~20 times a second as a
non-connectable BLE beacon (layout: firmware/libraries/PostureLink/Beacon.h;
keep BEACON_VER and the fields in sync). This turns each beacon into the
same lines the chair prints on USB (header, rows, '#' events), so
live_posture.py's ChairReader logic — and mostra.py --ble — work unchanged.

What the beacon doesn't carry comes out as unknown: absolute pitch/roll
(nan), the body's boot id, reference id and decision join (bcalid, bseq,
bage = -1). seq is the beacon count (body up), not the body's packet seq.
The link is one-way: send() can't reach the chair (calibrate with its
button; without a USB host it clears the calibration itself when the seat
stays empty). Beacons are best effort: losses are skipped rows.

Stats once per second, as an event and in `link`:
    # ble beacons=N lost=L rssi=R
(beacons received / missing from the sequence, in the last second).

    .venv/bin/python analysis/ble_chair.py                 # CSV to stdout
    .venv/bin/python analysis/ble_chair.py --log run.csv   # also to a file
    .venv/bin/python analysis/mostra.py --ble              # the exhibit screen

Needs Bluetooth on (rfkill unblock bluetooth; bluetoothctl power on) and
`bleak` (analysis/requirements.txt).
"""

from __future__ import annotations

import argparse
import asyncio
import math
import struct
import sys
import threading
import time
from collections import deque
from pathlib import Path

BEACON_VER = 1
BEACON_LEN = 27
COMPANY = 0xFFFF
NAN16 = -32768
# flags (Beacon.h BK_*)
BK_CAL, BK_ALERT, BK_BODY, BK_CAPTURE, BK_MUTE, BK_RADIO, BK_REJECT, BK_USB = (1 << i for i in range(8))
# bflags: bits 0-1 bctl, then BB_*
BB_ALERT, BB_MOTOR, BB_TEST, BB_INH, BB_MUTE, BB_CAL = (1 << i for i in range(2, 8))
SILENT_S = 2.0                 # no beacon this long: chair "disconnected"

HEADER = ("t_ms,cal,calid,dev,alert,dpitch,droll,pitch,roll,boot,seq,"
          "bctl,bcal,bcalid,balert,bmotor,btest,bseq,bage,imu,binh,vbat,"
          + ",".join(f"c{c}" for c in range(8, 16)) + ",reason,mute,bmute")


def decode(data: bytes) -> dict | None:
    """Beacon payload (manufacturer data after the company id) -> fields; None
    if it isn't one of ours."""
    if len(data) != BEACON_LEN or data[0] != BEACON_VER:
        return None
    _, seq, t16, flags, bflags, calid, reason, imu, dp, dr, vbat = struct.unpack_from("<BBHBBBbbhhH", data)
    fsr = []
    for i in range(4):
        v = int.from_bytes(data[15 + 3 * i:18 + 3 * i], "little")
        fsr += [v & 0xFFF, v >> 12]
    return dict(seq=seq, t16=t16, flags=flags, bflags=bflags, calid=calid, reason=reason, imu=imu,
                dpitch=math.nan if dp == NAN16 else dp / 10.0,
                droll=math.nan if dr == NAN16 else dr / 10.0,
                vbat=vbat, fsr=fsr)


def fmt(v: float) -> str:
    return "nan" if math.isnan(v) else f"{v:.1f}"


class Beacons:
    """Beacons -> chair CSV lines (and transition events), stateful: unwraps
    the 16-bit chair clock, numbers the rows, notices state changes."""

    def __init__(self):
        self.t_ms = None                       # unwrapped chair millis
        self.t16 = None
        self.wall = None
        self.seq = None                        # last beacon seq (8 bit)
        self.rows = 0                          # body-up rows: the `seq` column
        self.prev: dict | None = None
        self.count = self.lost = 0             # since the last stats()

    def lines(self, b: dict, wall: float) -> list[str]:
        out = []
        if self.seq is not None and wall - self.wall < SILENT_S:
            gap = (b["seq"] - self.seq) & 0xFF
            if gap == 0:
                return []                      # the same beacon again (another channel)
            self.lost += gap - 1
        if self.t16 is None or wall - self.wall > 60:   # first, or a gap the 16 bits can't span
            self.t_ms = b["t16"] if self.t_ms is None else self.t_ms + int((wall - self.wall) * 1000)
            out += ["# unit=raw", "# proto=ble" + str(BEACON_VER), HEADER]
        else:
            self.t_ms += (b["t16"] - self.t16) & 0xFFFF
        self.t16, self.wall, self.seq = b["t16"], wall, b["seq"]
        self.count += 1
        out += self._events(b)
        out.append(self._row(b))
        self.prev = b
        return out

    def _events(self, b: dict) -> list[str]:
        p, f = self.prev, b["flags"]
        pf = p["flags"] if p else 0
        ev = []
        rise = lambda m: f & m and not pf & m     # noqa: E731
        fall = lambda m: pf & m and not f & m     # noqa: E731
        if rise(BK_BODY):
            ev.append("# body link up (ble)")
        if fall(BK_BODY):
            ev.append("# body link lost")
        if rise(BK_CAPTURE):
            ev.append("# calibrating (hold upright 1 s), from ble")
        if f & BK_CAL and (not pf & BK_CAL or (p and p["calid"] != b["calid"])):
            ev.append(f"# calibrated upright: calid={b['calid']} (ble)")
        if fall(BK_CAL):
            ev.append(f"# calibration cleared calid={b['calid']}")
        if rise(BK_REJECT):
            ev.append("# calibrate rejected: seat rear unloaded (ble)")
        if rise(BK_ALERT) or fall(BK_ALERT):
            ev.append(("# alert ON  dev=" if f & BK_ALERT else "# alert OFF dev=") + fmt(self._dev(b)))
        if rise(BK_MUTE):
            ev.append("# motor muted (ble)")
        if fall(BK_MUTE):
            ev.append("# motor unmuted (ble)")
        if rise(BK_RADIO):
            ev.append("# radio init FAILED")
        return ev

    @staticmethod
    def _dev(b: dict) -> float:
        if not b["flags"] & BK_BODY or b["imu"] != 0:
            return math.nan
        return math.hypot(b["dpitch"], b["droll"]) if b["flags"] & BK_CAL else 0.0

    def _row(self, b: dict) -> str:
        f, bf, body = b["flags"], b["bflags"], b["flags"] & BK_BODY
        cols = [str(self.t_ms), str(int(bool(f & BK_CAL))), str(b["calid"]), fmt(self._dev(b)),
                str(int(bool(f & BK_ALERT)))]
        if body:
            self.rows += 1
            cols += [fmt(b["dpitch"]), fmt(b["droll"]), "nan", "nan", "0", str(self.rows & 0xFFFF),
                     str(bf & 3), str(int(bool(bf & BB_CAL))), "-1", str(int(bool(bf & BB_ALERT))),
                     str(int(bool(bf & BB_MOTOR))), str(int(bool(bf & BB_TEST))), "-1", "-1",
                     str(b["imu"]), str(int(bool(bf & BB_INH))), str(b["vbat"])]
        else:
            cols += ["nan"] * 4 + ["-1"] * 13
        cols += [str(v) for v in b["fsr"]]
        cols += [str(b["reason"]), str(int(bool(f & BK_MUTE))),
                 str(int(bool(bf & BB_MUTE))) if body else "-1"]
        return ",".join(cols)

    def stats(self, rssi: int | None) -> str:
        s = f"# ble beacons={self.count} lost={self.lost} rssi={rssi if rssi is not None else 'nan'}"
        self.count = self.lost = 0
        return s


async def scan(on_beacon, address: str | None = None, stop: threading.Event | None = None):
    """Calls on_beacon(payload_fields, rssi, wall_time) for every beacon seen."""
    from bleak import BleakScanner

    def cb(dev, adv):
        if address and dev.address.upper() != address.upper():
            return
        data = adv.manufacturer_data.get(COMPANY)
        b = decode(data) if data else None
        if b:
            on_beacon(b, adv.rssi, time.monotonic())

    # DuplicateData on: BlueZ reports every advertisement, not only changes
    async with BleakScanner(cb, scanning_mode="active", bluez={"filters": {"DuplicateData": True}}):
        while not (stop and stop.is_set()):
            await asyncio.sleep(0.2)


def _chair_reader_base():
    import live_posture as LP   # noqa: I001  (Qt backend; only when used as a ChairReader)
    return LP.ChairReader


class BleChairReader(_chair_reader_base()):
    """live_posture.ChairReader fed by the beacon instead of the serial port."""

    def __init__(self, log: Path | None, trail_n: int, address: str | None = None):
        super().__init__("ble", log, trail_n)
        self.address = address
        self.conv = Beacons()
        self.last_wall = 0.0
        self.rssi: deque[int] = deque(maxlen=20)
        self.error = ""

    def send(self, ch: str):
        pass                                   # one-way link: the chair's button calibrates

    def _beacon(self, b, rssi, wall):
        for line in self.conv.lines(b, wall):
            self._handle(line)
        self.last_wall = wall
        self.rssi.append(rssi)
        self.connected = True

    def _tick(self):
        while True:
            time.sleep(1.0)
            if time.monotonic() - self.last_wall > SILENT_S:
                self.connected = False
            rssi = round(sum(self.rssi) / len(self.rssi)) if self.rssi else None
            self.rssi.clear()
            line = self.conv.stats(rssi)
            with self.lock:
                self.link = line[2:]
            if self.log:
                self._handle(line)

    def run(self):
        threading.Thread(target=self._tick, daemon=True).start()
        while True:
            try:
                asyncio.run(scan(self._beacon, self.address))
            except Exception as e:             # adapter off / BlueZ error: retry
                self.connected = False
                if str(e) != self.error:
                    self.error = str(e)
                    with self.lock:
                        self.events.append(f"ble: {e}")
                    print(f"ble: {e}", file=sys.stderr)
                time.sleep(2.0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--log", type=Path, help="also write the CSV here (new file)")
    ap.add_argument("--address", help="only this chair (BLE address, as bluetoothctl shows it)")
    args = ap.parse_args()
    log = open(args.log, "x") if args.log else None
    conv = Beacons()
    rssi: list[int] = []
    last = [time.monotonic()]

    def out(line):
        print(line, flush=True)
        if log:
            log.write(line + "\n")
            log.flush()

    def on_beacon(b, r, wall):
        for line in conv.lines(b, wall):
            out(line)
        rssi.append(r)
        if wall - last[0] >= 1.0:
            last[0] = wall
            out(conv.stats(round(sum(rssi) / len(rssi)) if rssi else None))
            rssi.clear()

    try:
        asyncio.run(scan(on_beacon, args.address))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
