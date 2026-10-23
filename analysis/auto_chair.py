"""Exhibit reader: prefer fresh USB samples, fall back to BLE, return to USB.

Both readers stay running so a cable change needs no restart or BLE scan startup.
Only the selected source contributes samples. The exhibit uses a local monotonic
clock: USB has the chair's full uptime, BLE only its low 16 bits, so their CSV
clocks must never be spliced directly. This is a display feed, not a capture log.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass

import live_posture as LP
import ble_chair as B

USB_SILENT_S = 1.0


@dataclass
class Sample:
    number: int
    wall: float
    row: dict
    fsr: dict[int, float]
    raw: bool


class _Samples:
    """Observe parsed data, including FSR-only rows while the body is silent."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.samples: deque[Sample] = deque(maxlen=200)
        self.sample_number = 0

    def _handle(self, line):
        super()._handle(line)
        if not line[:1].isdigit():
            return
        with self.lock:
            parsed = LP.parse_row(line, self.fields, self.tail)
            if parsed is None or parsed[1] is None or not math.isfinite(parsed[0].get("t_ms", math.nan)):
                return
            row, fsr = parsed
            self.sample_number += 1
            self.samples.append(Sample(self.sample_number, time.monotonic(), row,
                                       dict(zip(LP.M.CHANNELS, fsr)), self.raw))


class _UsbReader(_Samples, LP.ChairReader):
    def _open_port(self):
        port = super()._open_port()
        with self.lock:
            self.samples.clear()                 # reconnect must deliver new data before winning
        return port


class _BleReader(_Samples, B.BleChairReader):
    pass


class AutoChairReader(LP.ChairReader):
    """One ChairReader interface for the exhibit; USB always has priority.

    Commands are sent only over the active USB connection, never queued for
    later. A missing Bluetooth adapter does not interrupt USB operation.
    """

    def __init__(self, port: str, trail_n: int, address: str | None = None):
        super().__init__(port, None, trail_n)
        self.usb = _UsbReader(port, None, trail_n)
        self.ble = _BleReader(None, trail_n, address=address)
        self.transport: str | None = None
        self._seen = {"USB": 0, "BLE": 0}
        self._origin = time.monotonic()
        self._last_wall: float | None = None
        self._last_chair_t: int | None = None
        self._previous_transport: str | None = None

    def send(self, ch: str):
        with self.lock:
            if self.transport == "USB" and self.usb.connected:
                self.usb.send(ch)

    def _poll(self, now: float):
        with self.lock:
            self.connected = False
            for name, source, timeout in (("USB", self.usb, USB_SILENT_S),
                                           ("BLE", self.ble, B.SILENT_S)):
                with source.lock:
                    if not source.connected or not source.samples or now - source.samples[-1].wall > timeout:
                        continue
                    switched = name != self.transport
                    if switched and self._last_wall is not None and source.samples[-1].wall <= self._last_wall:
                        continue                     # wait for a fallback newer than the last shown row
                    samples = ([source.samples[-1]] if switched else
                               [s for s in source.samples if s.number > self._seen[name]])
                    self.link = source.link
                    self.events = deque(source.events, maxlen=3)
                    event = source.cal_event
                    if event and (self.cal_event is None or event[1] > self.cal_event[1]):
                        self.cal_event = event        # a source switch must not replay an old clear
                self.transport, self.connected = name, True
                for sample in samples:
                    self._seen[name] = sample.number
                    self._accept(sample, name)
                return
            self.transport, self.body_up = None, False

    def _accept(self, sample: Sample, name: str):
        if self._last_wall is not None and sample.wall <= self._last_wall:
            return                                  # a cached fallback sample predates the USB row
        chair_t = int(sample.row["t_ms"]) & 0xFFFF
        switching = name != self._previous_transport
        if switching and self._last_chair_t is not None:
            # USB and its beacon may describe the same sample, 0–1 ms apart.
            if ((chair_t - self._last_chair_t) & 0xFFFF) in (0, 1, 0xFFFF):
                return
        row = dict(sample.row, t_ms=(sample.wall - self._origin) * 1000,
                   _transport=name, _max_dt=B.SILENT_S if name == "BLE" else 0.25)
        self._last_wall, self._last_chair_t = sample.wall, chair_t
        self._previous_transport = name
        self.raw, self.raw_fsr = sample.raw, dict(sample.fsr)
        self.fsr.append((sample.wall, {c: self._to_us(v) for c, v in sample.fsr.items()}))
        self.body_up = row["seq"] >= 0
        if self.body_up:
            self.rows.append(row)

    def run(self):
        self.usb.start()
        self.ble.start()
        while True:
            self._poll(time.monotonic())
            time.sleep(0.05)
