"""Exhibit view (science fair) of the posture monitor: one person at a time,
many people a day.

Same serial stream as live_posture.py (the chair unit, firmware/chair), but
built for visitors, in PT-BR: a big status band (good / attention / alert, with
the posture's name), "sua coluna" (the IMU deviation target, with where the
recorded postures fall), "sua cadeira" (the 8 FSRs as load, left/right
balance), how it works, the current visitor's numbers + a 2-minute timeline,
and the day's numbers.

Visitor flow (high turnover): put the collar on, sit upright, press the chair
button (or c / space here). A visit starts at that calibration; a recalibration
while still seated keeps the same visit. It ends when the seat stays empty for
LEAVE_S (or n): the chair's calibration is then cleared (no buzzing the next
person with someone else's reference; --keep-cal to skip) and the visit goes
into the day's numbers if it lasted MIN_VISIT_S. Day numbers persist per date
in --stats-dir, so restarting the app keeps them.

Keys:  c / space = calibrate   n = next visitor   r = clear calibration
       f = fullscreen   q = quit

    .venv/bin/python analysis/mostra.py
    .venv/bin/python analysis/mostra.py --window      # not fullscreen
    .venv/bin/python analysis/mostra.py --replay data/validation/<file>.csv   # no hardware
    .venv/bin/python analysis/mostra.py --ble         # wireless: the chair's BLE beacon

With --ble (ble_chair.py) the laptop only listens: calibrate with the chair's
button; with no USB host the chair clears the calibration itself when the seat
stays empty, as `n` would.

Diagnostics stay in live_posture.py; here only a small operator footer.
"""

from __future__ import annotations

import argparse
import datetime
import json
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path

import live_posture as LP  # noqa: I001  (sets the Qt backend first)
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.colors import LinearSegmentedColormap, ListedColormap
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle

import chair_map as M

# Fusion.h reason codes and seat-empty threshold (keep in sync)
R_OK, R_IMU_DEV, R_SLUMP, R_LEAN, R_RECLINE_OK, R_EMPTY, R_INVALID = range(7)
EMPTY_RAW = 2000.0

LEAVE_S = 10.0                 # seat empty this long: the visit is over
MIN_VISIT_S = 15.0             # seated time for a visit to count in the day's numbers
TIMELINE_S = 120.0
TIMELINE_BIN_S = 0.25
CALIBRATING_MAX_S = 4.0        # a capture is 1 s; past this, stop showing it
CALIBRATED_SHOW_S = 2.5
STATS_DIR = Path.home() / ".local" / "state" / "tcc-mostra"

# sample categories (timeline + tiles)
NONE, GOOD, WARN, ALERT = 0, 1, 2, 3

# stand identity (docs/stand/README.md). Status colours are the stand's green /
# pink stepped down for white text (bold, large: >= 3:1 — 3.4 and 4.9); the
# amber "attention" isn't on the panels but is a distinct third state (navy
# text, 6.3:1). Neutral states use the stand's gradient (white text >= 3.7:1).
GOOD_C, WARN_C, ALERT_C = "#2e9e64", "#fab219", "#c2416f"
INFO_C = "#1e71e4"                            # 4.1:1 on SURFACE (the cyan is only 2.3)
IDLE = None                                   # header: the stand gradient
STAND_GRADIENT = ["#1e71e4", "#9142d4", "#cd4fd3"]
INK, INK2, MUTED = "#003482", "#41557f", "#5b6890"   # navy; 6.6 and 4.9:1 on SURFACE
SURFACE, PLATE, PLATE_EC, EMPTY_FC = "#f2f1f6", "#ffffff", "#cccef0", "#e4e3ec"
REF_C = "#b9bcd6"                             # recorded postures, behind the live dot
CAT_CMAP = ListedColormap([EMPTY_FC, GOOD_C, WARN_C, ALERT_C])
LOAD_CMAP = LinearSegmentedColormap.from_list("load", ["#cfeafe", "#32a8fb", "#6a73e6", "#9142d4"])
REF_LABELS = {                                # (offset pt, ha): keep nearby postures apart
    "escorregado": ((-12, 8), "right"), "reclinado": ((12, -4), "left"),
    "cabeca": ((-8, -12), "right"),
}


# --- pure logic ---------------------------------------------------------------
def seat_raw(raw: dict[int, float], fmap: dict[str, int]) -> float | None:
    """Seat FSR sum (raw ADC) as Fusion.h's empty test sees it; None if unknown."""
    chans = [fmap.get(s) for s in M.SLOTS if M.SLOTS[s][0] == "seat"]
    if not raw or None in chans:
        return None
    return sum(raw.get(c, 0.0) for c in chans)


def category(row: dict) -> int:
    """Good / attention (bad, not yet held long enough) / alert / none."""
    reason = int(row.get("reason", -1))
    if not row.get("cal") or reason in (R_EMPTY, R_INVALID) or not LP.imu_valid(row):
        return NONE
    if row.get("alert"):
        return ALERT
    if reason in (R_IMU_DEV, R_SLUMP, R_LEAN):
        return WARN
    if reason == -1 and row.get("dev", 0) > LP.ALERT_DEG:   # firmware without `reason`
        return WARN
    return GOOD


def side(dp: float) -> str:
    return "direita" if dp > 0 else "esquerda"


def posture_name(row: dict) -> str:
    """What the person is doing, in plain words ('' when nothing to say).
    Collar mount: droll = forward (−) / back (+), dpitch = left (−) / right (+)."""
    reason = int(row.get("reason", -1))
    dp, dr = row.get("dpitch", 0.0), row.get("droll", 0.0)
    if reason == R_SLUMP:
        return "escorregado no assento"
    if reason == R_LEAN:
        return f"inclinado para a {side(dp)}"
    if reason == R_RECLINE_OK:
        return "reclinado no encosto"
    if reason == R_IMU_DEV or (reason == -1 and row.get("dev", 0) > LP.ALERT_DEG):
        if abs(dr) >= abs(dp):
            return "curvado para a frente" if dr < 0 else "inclinado para trás"
        return f"inclinado para a {side(dp)}"
    return ""


def fmt_dur(s: float) -> str:
    s = int(s)
    return f"{s // 60}:{s % 60:02d}"


@dataclass
class Visit:
    calid: int
    seated_s: float = 0.0
    good_s: float = 0.0
    alerts: int = 0
    streak_s: float = 0.0
    best_streak_s: float = 0.0
    in_alert: bool = False

    @property
    def good_pct(self) -> float | None:
        return 100.0 * self.good_s / self.seated_s if self.seated_s > 0 else None


@dataclass
class DayStats:
    date: str
    visitors: int = 0
    alerts: int = 0
    good_pcts: list[float] = field(default_factory=list)

    def add(self, v: Visit):
        self.visitors += 1
        self.alerts += v.alerts
        if v.good_pct is not None:
            self.good_pcts.append(round(v.good_pct, 1))

    @property
    def mean_pct(self) -> float | None:
        return float(np.mean(self.good_pcts)) if self.good_pcts else None

    @property
    def best_pct(self) -> float | None:
        return max(self.good_pcts) if self.good_pcts else None

    @classmethod
    def load(cls, folder: Path, date: str) -> DayStats:
        p = folder / f"{date}.json"
        try:
            return cls(**json.loads(p.read_text()))
        except (OSError, ValueError, TypeError):
            return cls(date)

    def save(self, folder: Path):
        try:
            folder.mkdir(parents=True, exist_ok=True)
            (folder / f"{self.date}.json").write_text(json.dumps(asdict(self)))
        except OSError as e:
            print(f"day stats not saved: {e}")


class Exhibit:
    """Per-visitor numbers from the chair rows (see the module docstring)."""

    def __init__(self, day: DayStats, on_day_change=None, max_dt: float = 0.25):
        self.day, self.on_day_change = day, on_day_change
        self.max_dt = max_dt                   # tolerated sample gap (BLE needs more than USB)
        self.visit: Visit | None = None
        self.last: Visit | None = None           # the previous visitor's, shown until the next one
        self.prev_t: float | None = None
        self.ended_calid: int | None = None      # its calibration can't start a new visit
        self.empty_since: float | None = None
        self.timeline: deque[tuple[float, int]] = deque()

    def feed(self, row: dict, seated: bool | None = None):
        """One chair row (body up). seated: from the seat FSRs; None = unknown."""
        t = row["t_ms"] / 1000.0
        if self.prev_t is not None and t < self.prev_t:   # chair reset / replay loop
            self.timeline.clear()
        dt = 0.0 if self.prev_t is None else min(max(t - self.prev_t, 0.0), self.max_dt)
        self.prev_t = t
        cat = category(row)
        if seated is False:
            cat = NONE
        cal, calid = bool(row.get("cal")), int(row.get("calid", -1))
        if not cal:
            self.end()                             # chair cleared it: authoritative visit boundary
        if cal and calid != self.ended_calid:
            if self.visit is None:
                self.visit = Visit(calid)
                dt = 0.0                           # don't count time before this visitor calibrated
            self.visit.calid = calid                # recalibrating, same person
        v = self.visit
        if v is not None and cat != NONE:
            v.seated_s += dt
            if cat == GOOD:
                v.good_s += dt
                v.streak_s += dt
                v.best_streak_s = max(v.best_streak_s, v.streak_s)
        if v is not None and cat != GOOD:            # away / invalid breaks "sem parar" too
            v.streak_s = 0.0
        if v is not None:
            if cat == ALERT and not v.in_alert:
                v.alerts += 1
            v.in_alert = cat == ALERT
        self.timeline.append((t, cat if v is not None else NONE))
        while self.timeline and self.timeline[0][0] < t - TIMELINE_S:
            self.timeline.popleft()

    def seat(self, seated: bool | None, now: float) -> bool:
        """Seat state at monotonic `now`; True when that just ended the visit."""
        if self.visit is None or seated is not False:
            self.empty_since = None
            return False
        if self.empty_since is None:
            self.empty_since = now
        if now - self.empty_since >= LEAVE_S:
            self.end()
            return True
        return False

    def end(self):
        v, self.visit, self.empty_since = self.visit, None, None
        if v is None:
            return
        self.ended_calid = v.calid
        if v.seated_s >= MIN_VISIT_S:
            self.last = v
            self.day.add(v)
            if self.on_day_change:
                self.on_day_change(self.day)

    def strip(self, now_t: float | None = None) -> np.ndarray:
        """Timeline as category bins over the last TIMELINE_S (oldest first)."""
        n = int(TIMELINE_S / TIMELINE_BIN_S)
        out = np.zeros(n, dtype=int)
        if not self.timeline:
            return out
        end = self.timeline[-1][0] if now_t is None else now_t
        samples = list(self.timeline)
        for j, (t, cat) in enumerate(samples):
            until = samples[j + 1][0] if j + 1 < len(samples) else t
            if self.max_dt > TIMELINE_BIN_S and 0 <= until - t <= self.max_dt:
                for fill_t in np.arange(t, until, TIMELINE_BIN_S):
                    k = n - 1 - int((end - fill_t) / TIMELINE_BIN_S)
                    if 0 <= k < n:
                        out[k] = max(out[k], cat)
            i = n - 1 - int((end - t) / TIMELINE_BIN_S)
            if 0 <= i < n:
                out[i] = max(out[i], cat)
        return out


class ReplayReader(LP.ChairReader):
    """A recorded capture played back at its own pace (t_ms), looping: for
    testing the exhibit without the hardware, or as a stand-in if it fails."""

    def __init__(self, path: Path, speed: float = 1.0):
        super().__init__(str(path), None, trail_n=200)
        self.path, self.speed = path, speed

    def send(self, ch: str):
        pass

    def run(self):
        lines = self.path.read_text().splitlines()
        while True:
            self.connected = True
            t0_row = t0 = None
            for line in lines:
                if line[:1].isdigit():
                    t_ms = int(line.split(",", 1)[0])
                    if t0_row is None:
                        t0_row, t0 = t_ms, time.monotonic()
                    wait = t0 + (t_ms - t0_row) / 1000.0 / self.speed - time.monotonic()
                    if wait > 0:
                        time.sleep(wait)
                self._handle(line)
            with self.lock:
                self.rows.clear()


# --- drawing ------------------------------------------------------------------
def panel_title(fig, x, y, text, sub=""):
    fig.text(x, y, text.upper(), fontsize=20, weight="bold", color=INK, va="bottom")
    if sub:
        fig.text(x, y - 0.005, sub, fontsize=11, color=MUTED, va="top")


class Tile:
    """Hero-number tile: label above, big value, small note below."""

    def __init__(self, fig, rect, label: str):
        x, y, w, h = rect
        fig.patches.append(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.012",
                                          transform=fig.transFigure, fc="white", ec=PLATE_EC,
                                          lw=1, zorder=0))
        self.label = fig.text(x + 0.012, y + h - 0.016, label, fontsize=12, color=INK2, va="top")
        self.value = fig.text(x + 0.012, y + h * 0.42, "—", fontsize=34, weight="bold",
                              color=INK, va="center")
        self.note = fig.text(x + 0.012, y + 0.014, "", fontsize=10, color=MUTED, va="bottom")

    def set(self, value: str, note: str = "", label: str | None = None):
        self.value.set_text(value)
        self.note.set_text(note)
        if label is not None:
            self.label.set_text(label)


class ChairPanel:
    """The chair from above, backrest folded back: 4 + 4 spots coloured by load."""

    PLATES = {"back": (0.0, 11.0), "seat": (0.0, 0.0)}
    SPOT_R = 1.9

    def __init__(self, fig, rect, fmap):
        self.fmap = fmap
        ax = self.ax = fig.add_axes(rect)
        ax.set_xlim(-3.5, 13.5)
        ax.set_ylim(-3.2, 22)
        ax.set_aspect("equal")
        ax.axis("off")
        names = {"back": "encosto", "seat": "assento  (↓ borda da frente)"}
        self.spots, self.cop = {}, {}
        for plate, (x0, y0) in self.PLATES.items():
            ax.add_patch(FancyBboxPatch((x0, y0), 10, 10, boxstyle="round,pad=0,rounding_size=1.4",
                                        fc=PLATE, ec=PLATE_EC, lw=1.2, zorder=0))
            ax.text(x0 + 5, y0 + 10.45 if plate == "back" else y0 - 0.5, names[plate],
                    ha="center", va="bottom" if plate == "back" else "top", fontsize=12, color=INK2)
            self.cop[plate], = ax.plot([], [], marker="+", ms=22, mew=3, color=INK, zorder=5)
        ax.text(-1.2, 10.5, "E", ha="center", va="center", fontsize=13, color=MUTED, weight="bold")
        ax.text(11.2, 10.5, "D", ha="center", va="center", fontsize=13, color=MUTED, weight="bold")
        for slot, (plate, _, sx, sy) in M.SLOTS.items():
            x0, y0 = self.PLATES[plate]
            self.spots[slot] = ax.add_patch(Circle((x0 + 2.5 + 5 * sx, y0 + 2.5 + 5 * sy),
                                                   self.SPOT_R, zorder=2, lw=2, ec=PLATE))
        self.balance = ax.text(5, -2.2, "", ha="center", va="top", fontsize=13, color=INK)

    def update(self, g: dict[int, float], live: bool):
        for slot, spot in self.spots.items():
            us = g.get(self.fmap.get(slot, -1), 0.0) if live else 0.0
            spot.set_facecolor(LOAD_CMAP(float(LP.LOAD_NORM(us))) if us > 0 else EMPTY_FC)
        for plate, (x0, y0) in self.PLATES.items():
            bal = M.plate_balance(self.fmap, g if live else {}, plate)
            self.cop[plate].set_data(*(([x0 + 2.5 + 5 * bal[0]], [y0 + 2.5 + 5 * bal[1]])
                                       if bal else ([], [])))
        seat = M.plate_balance(self.fmap, g if live else {}, "seat")
        self.balance.set_text(f"peso no assento:  esquerda {100 * (1 - seat[0]):.0f} %  ·  "
                              f"direita {100 * seat[0]:.0f} %" if seat else "assento vazio")


HOW = [
    ("1", "Colar", "um sensor de movimento (IMU)\nno pescoço mede a inclinação\ndas costas."),
    ("2", "Cadeira", "8 sensores de pressão mostram\ncomo o peso se distribui no\nassento e no encosto."),
    ("3", "Decisão", "a cadeira junta os dois; se a\npostura ruim durar 3 s, o colar\nvibra até você corrigir."),
    ("✓", "Calibração", "cada pessoa senta reta e aperta\no botão: o sistema aprende a\nSUA postura, não um padrão."),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default=LP.CHAIR_PORT)
    ap.add_argument("--map", type=Path, default=M.MAP_PATH, help="FSR channel -> chair spot map")
    ap.add_argument("--window", action="store_true", help="start windowed, not fullscreen")
    ap.add_argument("--keep-cal", action="store_true",
                    help="don't clear the chair's calibration when a visitor leaves")
    ap.add_argument("--stats-dir", type=Path, default=STATS_DIR,
                    help="where the day's numbers are kept (one JSON per date)")
    ap.add_argument("--no-ref", action="store_true", help="don't draw the recorded postures")
    ap.add_argument("--replay", type=Path,
                    help="play a recorded capture instead of the serial port (day numbers in memory only)")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed")
    ap.add_argument("--ble", action="store_true",
                    help="read the chair's BLE beacon instead of the serial port (no USB)")
    ap.add_argument("--ble-address", help="with --ble: only this chair's BLE address")
    ap.add_argument("--screenshot", type=Path, help="save the screen here after --at s and quit")
    ap.add_argument("--at", type=float, default=5.0, help="seconds before --screenshot")
    args = ap.parse_args()

    fmap = M.load_map(args.map)
    today = datetime.date.today().isoformat()
    if args.replay:                              # a replay never touches the real day's numbers
        exhibit = Exhibit(DayStats(today))
    else:
        exhibit = Exhibit(DayStats.load(args.stats_dir, today),
                          on_day_change=lambda d: d.save(args.stats_dir),
                          max_dt=2.0 if args.ble else 0.25)
    ref_runs = [] if args.no_ref else LP.load_reference_runs()
    if args.replay:
        reader = ReplayReader(args.replay, args.speed)
    elif args.ble:
        import ble_chair
        reader = ble_chair.BleChairReader(None, trail_n=200, address=args.ble_address)
    else:
        reader = LP.ChairReader(args.port, None, trail_n=200)
    reader.start()

    for k in ("keymap.back", "keymap.home", "keymap.save", "keymap.pan", "keymap.zoom",
              "keymap.grid", "keymap.yscale", "keymap.xscale"):
        plt.rcParams[k] = []
    plt.rcParams["font.family"] = ["Noto Sans", "DejaVu Sans"]   # closer to the panels; DejaVu for ✓ ▲ ■ ←
    fig = plt.figure(figsize=(16, 10), facecolor=SURFACE)
    fig.canvas.manager.set_window_title("Monitor de postura — mostra")
    try:
        fig.canvas.manager.toolbar.hide()
    except AttributeError:
        pass

    # header band
    head = fig.add_axes((0, 0.855, 1, 0.145))
    head.set_xlim(0, 1)
    head.set_ylim(0, 1)
    head.axis("off")
    head.imshow(LinearSegmentedColormap.from_list("stand", STAND_GRADIENT)(np.linspace(0, 1, 512))[None],
                aspect="auto", extent=(0, 1, 0, 1), zorder=0)
    band = head.add_patch(Rectangle((0, 0), 1, 1, lw=0, zorder=1))
    status = head.text(0.03, 0.60, "", fontsize=32, weight="bold", color="white", va="center")
    status_sub = head.text(0.03, 0.20, "", fontsize=18, color="white", va="center")
    _ = [head.text(0.97, 0.62, "Monitor de postura", fontsize=15, weight="bold",
                       color="white", ha="right", va="center", alpha=0.9),
             head.text(0.97, 0.32, "colar + cadeira com sensores", fontsize=11,
                       color="white", ha="right", va="center", alpha=0.85)]

    # sua coluna
    panel_title(fig, 0.03, 0.795, "Sua coluna", "desvio da sua postura calibrada (centro)")
    ax = fig.add_axes((0.03, 0.30, 0.36, 0.47))
    ax.add_patch(Circle((0, 0), LP.ALERT_DEG, fc=GOOD_C, alpha=0.10, lw=0, zorder=0))
    ax.add_patch(Circle((0, 0), LP.ALERT_DEG, fill=False, ec=GOOD_C, lw=1.5, alpha=0.6))
    ax.text(-0.72 * LP.ALERT_DEG, 0.72 * LP.ALERT_DEG, "zona boa", ha="right", va="bottom",
            fontsize=11, color=GOOD_C, weight="bold")
    ax.axhline(0, color="#e1e0d9", lw=0.8, zorder=0)
    ax.axvline(0, color="#e1e0d9", lw=0.8, zorder=0)
    ax.set_xlim(-40, 40)
    ax.set_ylim(-50, 30)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color(PLATE_EC)
    ax.set_facecolor("white")
    for x, y, txt, ha, va in ((0, 29, "↑ trás", "center", "top"), (0, -49, "↓ frente", "center", "bottom"),
                              (-39, 0.6, "← esquerda", "left", "bottom"),
                              (39, 0.6, "direita →", "right", "bottom")):
        ax.text(x, y, txt, ha=ha, va=va, fontsize=11, color=MUTED)
    if ref_runs:
        for key, pts in LP.reference_points(ref_runs).items():
            mx, my = np.median(pts, axis=0)
            label = LP.REF_POSTURES[key][1]
            if np.hypot(mx, my) < LP.ALERT_DEG:          # crossed legs: inside the good zone
                continue
            ax.scatter([mx], [my], s=70, color=REF_C, edgecolor="white", lw=1.5, zorder=2)
            off, ha = REF_LABELS.get(key, LP.REF_POSTURES[key][2:])
            ax.annotate(label, (mx, my), xytext=off, textcoords="offset points", ha=ha,
                        va="center", fontsize=10, color=MUTED, zorder=2)
    trail, = ax.plot([], [], color=INK, lw=2.5, alpha=0.35, zorder=4, solid_capstyle="round")
    dot = ax.scatter([0], [0], s=520, color=GOOD_C, edgecolor="white", lw=3, zorder=6)
    steps = ax.text(0.5, 0.5, "", transform=ax.transAxes, ha="center", va="center", fontsize=17,
                    color=INK, linespacing=1.8, zorder=8,
                    bbox=dict(boxstyle="round,pad=0.8", fc="white", ec=PLATE_EC, alpha=0.95))

    # sua cadeira
    panel_title(fig, 0.43, 0.795, "Sua cadeira", "vista de cima · cor = pressão · + = centro do peso")
    chair = ChairPanel(fig, (0.42, 0.29, 0.25, 0.45), fmap)

    # como funciona
    panel_title(fig, 0.71, 0.795, "Como funciona")
    for i, (num, title, body) in enumerate(HOW):
        y = 0.735 - i * 0.112
        fig.text(0.715, y, num, fontsize=22, weight="bold", color=INFO_C, va="top", ha="center")
        fig.text(0.735, y, title, fontsize=14, weight="bold", color=INK, va="top")
        fig.text(0.735, y - 0.03, body, fontsize=11, color=INK2, va="top", linespacing=1.15)

    # você (current / last visitor) + timeline
    who = fig.text(0.03, 0.262, "", fontsize=16, weight="bold", color=INK, va="bottom")
    tw, gap = 0.148, 0.012
    tiles = [Tile(fig, (0.03 + i * (tw + gap), 0.115, tw, 0.14), lab) for i, lab in
             enumerate(("Tempo sentado", "Postura boa", "Alertas (vibrações)", "Maior sequência boa"))]
    tl = fig.add_axes((0.03, 0.058, 4 * tw + 3 * gap, 0.035))
    strip_img = tl.imshow(np.zeros((1, int(TIMELINE_S / TIMELINE_BIN_S))), aspect="auto",
                          cmap=CAT_CMAP, vmin=-0.5, vmax=3.5, interpolation="nearest",
                          extent=(-TIMELINE_S, 0, 0, 1))
    tl.set_yticks([])
    tl.set_xticks([-120, -90, -60, -30, 0], ["−2 min", "−1:30", "−1 min", "−0:30", "agora"],
                  fontsize=9, color=MUTED)
    tl.tick_params(length=0)
    for s in tl.spines.values():
        s.set_visible(False)
    for x, c, label in ((0.03, GOOD_C, "boa"), (0.07, WARN_C, "atenção"), (0.125, ALERT_C, "alerta"),
                        (0.175, EMPTY_FC, "fora / sem calibração")):
        fig.text(x, 0.097, "■", color=c, fontsize=12, va="bottom")
        fig.text(x + 0.011, 0.097, label, color=INK2, fontsize=10, va="bottom")

    # hoje
    fig.text(0.71, 0.262, "HOJE NA MOSTRA", fontsize=16, weight="bold", color=INK, va="bottom")
    day_tiles = [Tile(fig, (0.71, 0.115, 0.13, 0.14), "Pessoas"),
                 Tile(fig, (0.85, 0.115, 0.13, 0.14), "Média do dia")]
    record = fig.text(0.71, 0.075, "", fontsize=13, color=INK, va="center")
    footer = fig.text(0.985, 0.006, "", fontsize=9, color=MUTED, ha="right", va="bottom")
    fig.text(0.015, 0.006, "sem fio: calibre pelo botão da cadeira · f tela cheia" if args.ble else
             "c / espaço calibrar · n próximo visitante · f tela cheia",
             fontsize=9, color=MUTED, va="bottom")

    last_t = [None]
    last_cal_event = [None]

    def calibrate():
        reader.send("c")

    def next_visitor():
        exhibit.end()
        if not args.keep_cal:
            reader.send("r")

    def on_key(ev):
        if ev.key in ("c", " "):
            calibrate()
        elif ev.key == "n":
            next_visitor()
        elif ev.key == "r":
            reader.send("r")
    fig.canvas.mpl_connect("key_press_event", on_key)

    def header(colour, title, sub="", ink="white"):
        band.set_visible(colour is not None)          # None: the stand gradient shows
        if colour is not None:
            band.set_facecolor(colour)
        status.set(text=title, color=ink)
        status_sub.set(text=sub, color=ink)

    def update(_):
        rows, fsr, _events, link, body_up = reader.snapshot()
        with reader.lock:
            raw, cal_event = dict(reader.raw_fsr), reader.cal_event
        now = time.monotonic()
        live = bool(fsr) and now - fsr[-1][0] < max(1.0, exhibit.max_dt)
        seat = seat_raw(raw, fmap) if live and reader.raw else None
        seated = None if seat is None else seat >= EMPTY_RAW

        for r in rows:                                  # new rows since the last frame
            if last_t[0] is None or r["t_ms"] > last_t[0]:
                exhibit.feed(r, seated)
        if rows:
            last_t[0] = rows[-1]["t_ms"]
        if cal_event != last_cal_event[0]:
            if cal_event and cal_event[0].startswith("calibration cleared"):
                exhibit.end()                      # also arrives while the body is silent (no rows)
            last_cal_event[0] = cal_event
        if exhibit.seat(seated, now) and not args.keep_cal:
            reader.send("r")
        if exhibit.day.date != datetime.date.today().isoformat():   # past midnight
            exhibit.day = DayStats(datetime.date.today().isoformat())

        chair.update(fsr[-1][1] if fsr else {}, live)
        body_up = body_up and bool(rows)
        r = rows[-1] if rows else None
        cat = category(r) if r is not None and body_up else NONE
        if seated is False:
            cat = NONE
        cal_txt, cal_age = cal_event if cal_event else ("", 1e9)
        cal_age = now - cal_age

        # status band
        if not reader.connected:
            header(IDLE, "Conectando à cadeira…", "a cadeira está ligada? (sem sinal BLE)" if args.ble
                   else "verifique o cabo USB da cadeira")
        elif cal_txt.startswith("calibrating") and cal_age < CALIBRATING_MAX_S:
            header(INFO_C, "Calibrando… fique parado", "sentado reto, olhando para a frente")
        elif cal_txt.startswith("calibrated upright") and cal_age < CALIBRATED_SHOW_S:
            header(GOOD_C, "Pronto!", "o sistema aprendeu a sua postura")
        elif cal_txt.startswith(("calibrate rejected", "calibrate ignored",
                                 "calibration aborted")) and cal_age < CALIBRATED_SHOW_S * 2:
            header(WARN_C, "Calibração não deu certo", "sente-se reto, fique parado e "
                   "aperte o botão de novo", ink=INK)
        elif not body_up:
            header(IDLE, "Ligue o colar", "o colar está sem sinal")
        elif not LP.imu_valid(r):
            header(IDLE, "Ajuste o colar", "o sensor do colar está sem leitura")
        elif seated is False:
            header(IDLE, "Sente-se para começar", "a cadeira está vazia")
        elif not r["cal"] or exhibit.visit is None:
            header(IDLE, "Sente-se reto e aperte o botão da cadeira",
                   "assim o sistema aprende a SUA postura")
        elif cat == ALERT:
            header(ALERT_C, "▲  Corrija a postura",
                   (posture_name(r) or "postura ruim").capitalize() + " · o colar está vibrando")
        elif cat == WARN:
            header(WARN_C, "●  Atenção", (posture_name(r) or "postura ruim").capitalize()
                   + " · se continuar, o colar vibra", ink=INK)
        else:
            name = posture_name(r)
            header(GOOD_C, "✓  Postura boa", name.capitalize() + " — tudo bem" if name else
                   "continue assim")

        # sua coluna
        show = body_up and r is not None and r["cal"] and LP.imu_valid(r) and seated is not False
        if show:
            pts = np.array([(x["dpitch"], x["droll"]) for x in rows[-60:] if x["cal"]
                            and LP.imu_valid(x)])
            trail.set_data(pts[:, 0], pts[:, 1])
            dot.set_offsets([np.clip(pts[-1], [-39, -49], [39, 29])])
            dot.set_color({GOOD: GOOD_C, WARN: WARN_C, ALERT: ALERT_C}.get(cat, MUTED))
            dot.set_edgecolor("white")
        trail.set_visible(bool(show))
        dot.set_visible(bool(show))
        if exhibit.visit is None and reader.connected:
            steps.set_text("1.  Coloque o colar\n2.  Sente-se reto, costas no encosto\n"
                           "3.  Aperte o botão da cadeira")
        else:
            steps.set_text("")
        steps.set_visible(bool(steps.get_text()))

        # você
        v = exhibit.visit or exhibit.last
        who.set_text("VOCÊ AGORA" if exhibit.visit else
                     "ÚLTIMA PESSOA" if exhibit.last else "VOCÊ")
        if v is None:
            for t in tiles:
                t.set("—")
        else:
            pct = v.good_pct
            tiles[0].set(fmt_dur(v.seated_s))
            tiles[1].set("—" if pct is None else f"{pct:.0f} %",
                         "" if pct is None else "do tempo sentado")
            tiles[2].set(str(v.alerts), "postura ruim por 3 s ou mais")
            tiles[3].set(fmt_dur(v.best_streak_s), "sem parar")
            tiles[1].value.set_color(GOOD_C if pct is not None and pct >= 80 else INK)
        strip = exhibit.strip(r["t_ms"] / 1000.0 if body_up and r is not None else None)
        strip_img.set_data(strip[None, :])

        # hoje
        d = exhibit.day
        day_tiles[0].set(str(d.visitors), f"{d.alerts} vibrações no total")
        day_tiles[1].set("—" if d.mean_pct is None else f"{d.mean_pct:.0f} %", "de postura boa")
        record.set_text("" if d.best_pct is None else
                        f"Recorde do dia:  {d.best_pct:.0f} % de postura boa")

        # operator footer
        bits = []
        if r is not None and body_up:
            bits.append(LP.battery_text(r).replace("bateria", "bateria do colar"))
        lost = next((w for w in link.split() if w.startswith("lost=")), "")
        if args.ble:
            n = next((w[8:] for w in link.split() if w.startswith("beacons=")), "")
            bits.append(f"BLE {n}/s" if reader.connected and n else "BLE sem sinal")
        elif lost:
            bits.append("enlace " + ("ok" if lost == "lost=0" else lost.replace("lost=", "perdas ")))
        footer.set_text("  ·  ".join(bits))

    _anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)  # noqa: F841
    if args.screenshot:
        def shoot():
            fig.savefig(args.screenshot, dpi=120, facecolor=SURFACE)
            plt.close(fig)
        t = fig.canvas.new_timer(interval=int(args.at * 1000))
        t.single_shot = True
        t.add_callback(shoot)
        t.start()
    elif not args.window:
        fig.canvas.manager.full_screen_toggle()
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
