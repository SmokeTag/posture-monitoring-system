"""Live view of the body unit's IMU in the calibrated decision space.

Reads the serial CSV of firmware/body/body.ino and plots the current
(dpitch, droll) point with a short trail, on top of the per-posture clusters
from a recorded capture (the right panel of figures/02_separation.png), with
the alert (10°) and clear (7°) circles. The status box shows dev, alert and the
LED/motor state as the firmware reports them.

Keys (plot window focused):  c = calibrate (sends 'c' to the body)   r = clear

    .venv/bin/python analysis/live_imu.py
    .venv/bin/python analysis/live_imu.py --port /dev/ttyACM1 --log data/<name>.csv

Needs pyserial + PyQt6 in the venv (see requirements.txt).
"""

from __future__ import annotations

import argparse
import threading
import time
from collections import deque
from pathlib import Path

import matplotlib

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import serial  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402

import postures as P  # noqa: E402

BODY_PORT = "/dev/serial/by-id/usb-Seeed_XIAO_nRF52840_Sense_3B4A158C694F1777-if00"
REF_CSV = P.DATA_DIR / "2026-06-19_andre_no-neck-mount_own-chair.csv"
ALERT_DEG, CLEAR_DEG = 10.0, 7.0            # keep in sync with body.ino
FIELDS = ["t_ms", "cal", "dev", "alert", "led", "dpitch", "droll", "pitch", "roll", "seq"]

# status colours (state, not identity) — always paired with a text label
C_OK, C_OVER, C_ALERT = "#444444", "#e69f00", "#d62728"


class BodyReader(threading.Thread):
    """Background serial reader; reconnects if the board resets."""

    def __init__(self, port: str, log: Path | None, trail_n: int):
        super().__init__(daemon=True)
        self.port, self.ser = port, None
        self.log = open(log, "a") if log else None
        self.lock = threading.Lock()
        self.rows: deque[dict] = deque(maxlen=trail_n)
        self.events: deque[str] = deque(maxlen=3)
        self.link = ""
        self.connected = False

    def send(self, ch: str):
        if self.ser:
            try:
                self.ser.write(ch.encode())
            except serial.SerialException:
                pass

    def run(self):
        while True:
            try:
                self.ser = serial.Serial(self.port, 115200, timeout=0.5)
                self.connected = True
                while True:
                    line = self.ser.readline().decode(errors="replace").strip()
                    if line:
                        self._handle(line)
            except (serial.SerialException, OSError):
                self.connected, self.ser = False, None
                time.sleep(1.0)

    def _handle(self, line: str):
        if self.log:
            self.log.write(line + "\n")
            self.log.flush()
        with self.lock:
            if line.startswith("# rx"):
                self.link = line[2:]
            elif line.startswith("#"):
                self.events.append(line[2:])
            else:
                parts = line.split(",")
                if len(parts) < len(FIELDS) or not parts[0].isdigit():
                    return
                try:
                    self.rows.append({k: float(v) for k, v in zip(FIELDS, parts)})
                except ValueError:
                    pass

    def snapshot(self):
        with self.lock:
            return list(self.rows), list(self.events), self.link


def draw_reference(ax, csv: Path):
    """Recorded postures, calibrated deviation (cal==1, steady rows)."""
    if not csv.exists():
        return
    st = P.steady(P.load(csv))
    st = st[st["cal"] == 1]
    for tag, g in st.groupby("tag"):
        c = P.TAG_COLORS.get(tag, "#999")
        ax.scatter(g["dpitch"], g["droll"], s=5, alpha=0.10, color=c, lw=0)
        mx, my = g["dpitch"].mean(), g["droll"].mean()
        ax.scatter([mx], [my], s=110, color=c, edgecolor="white", lw=2, zorder=3,
                   alpha=0.8, label=P.TAG_LABELS.get(tag, str(tag)))
        below = tag == 4                              # head-down sits beside slouch
        ax.annotate(P.TAG_LABELS.get(tag, str(tag)), (mx, my),
                    xytext=(8, -14 if below else 6),
                    textcoords="offset points", fontsize=8, color="#555")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default=BODY_PORT)
    ap.add_argument("--ref", type=Path, default=REF_CSV,
                    help="recorded capture drawn as reference clusters")
    ap.add_argument("--log", type=Path, help="also append every serial line here")
    ap.add_argument("--trail", type=float, default=3.0, help="trail length, s")
    args = ap.parse_args()

    reader = BodyReader(args.port, args.log, trail_n=max(2, int(args.trail * 20)))
    reader.start()

    for k in ("keymap.back", "keymap.home"):          # free 'c' and 'r'
        plt.rcParams[k] = [x for x in plt.rcParams[k] if x not in ("c", "r")]

    fig, ax = plt.subplots(figsize=(12, 8))
    fig.canvas.manager.set_window_title("Body IMU — live")
    draw_reference(ax, args.ref)
    ax.add_patch(Circle((0, 0), ALERT_DEG, fill=False, color="#888", lw=1.5))
    ax.add_patch(Circle((0, 0), CLEAR_DEG, fill=False, color="#888", lw=1, ls=":"))
    ax.annotate(f"alert {ALERT_DEG:.0f}°", (ALERT_DEG * 0.71, ALERT_DEG * 0.71),
                xytext=(4, 4), textcoords="offset points", fontsize=8, color="#666")
    ax.axhline(0, color="#ddd", lw=0.8, zorder=0)
    ax.axvline(0, color="#ddd", lw=0.8, zorder=0)
    ax.set_xlim(-40, 35)
    ax.set_ylim(-30, 45)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_xlabel("dpitch [deg]   (− forward · back +)")
    ax.set_ylabel("droll [deg]")
    ax.set_title("Live deviation from calibrated upright\n"
                 f"reference clusters: {args.ref.name} (earlier mount — indicative)",
                 fontsize=10)
    ax.legend(loc="lower left", fontsize=8, title="recorded posture", frameon=False)

    trail, = ax.plot([], [], color="#333", lw=2, alpha=0.5, zorder=4)
    dot = ax.scatter([0], [0], s=260, color=C_OK, edgecolor="white", lw=2.5, zorder=6)
    status = ax.text(1.03, 1.0, "", transform=ax.transAxes, ha="left", va="top",
                     family="monospace", fontsize=10,
                     bbox=dict(boxstyle="round", fc="white", ec="#ccc"))
    banner = ax.text(0.5, 0.5, "", transform=ax.transAxes, ha="center", va="center",
                     fontsize=13, color="#333",
                     bbox=dict(boxstyle="round", fc="white", ec="#ccc", alpha=0.9))

    def on_key(ev):
        if ev.key in ("c", "r"):
            reader.send(ev.key)
    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        rows, events, link = reader.snapshot()
        if not reader.connected:
            banner.set_text(f"waiting for {Path(args.port).name} …")
        elif not rows:
            banner.set_text("connected, no data yet …")
        elif not rows[-1]["cal"]:
            banner.set_text("NOT CALIBRATED\nsit upright, press the chair button "
                            "(or 'c' here)")
        else:
            banner.set_text("")
        banner.set_visible(bool(banner.get_text()))

        cal_rows = [r for r in rows if r["cal"]]
        if cal_rows:
            x = np.array([r["dpitch"] for r in cal_rows])
            y = np.array([r["droll"] for r in cal_rows])
            trail.set_data(x, y)
            dot.set_offsets([[x[-1], y[-1]]])
            dot.set_visible(True)
        else:
            trail.set_data([], [])
            dot.set_visible(False)

        if rows:
            r = rows[-1]
            if r["alert"]:
                colour, state = C_ALERT, "▲ ALERT"
            elif r["cal"] and r["dev"] > ALERT_DEG:
                colour, state = C_OVER, "● over threshold"
            else:
                colour, state = C_OK, "✓ ok" if r["cal"] else "— uncalibrated"
            dot.set_color(colour)
            dot.set_edgecolor("white")
            status.set_text(
                f"dev   {r['dev']:5.1f}°\n"
                f"state {state}\n"
                f"LED   {'● ON ' if r['led'] else '○ off'}\n"
                f"chair {link or '—'}\n"
                + "\nlast events\n" + "\n".join(f"· {e[:44]}" for e in events))
        return trail, dot, status, banner

    _anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)  # noqa: F841
    fig.subplots_adjust(left=0.07, right=0.62, top=0.92, bottom=0.08)
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
