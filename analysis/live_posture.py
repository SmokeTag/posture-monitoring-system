"""Live view of the body unit's IMU and the chair's 8 FSRs.

Reads the serial CSV of the chair unit (firmware/chair/chair.ino, the "brain",
which receives the body's IMU over ESB and scans its FSRs).

Left: the current (dpitch, droll) point with a short trail, with the alert
(10°) and clear (7°) circles, on top of the postures recorded in data/2026-10-21_*_run*.csv
(one dot per subject, a big one at their median; red = should alert,
green = must not; --no-ref hides them). The status box shows dev and the chair's
decision, then what the body reports it applied: who decides (chair /
fallback), the age of the decision it holds and whether it matches the chair
row that made it (✓ / ≠; "unconfirmed" when that can't be checked), and the
motor output (alert or motor test), and the body's battery voltage (a body
that goes silent right after a low reading most likely hit its low-battery
cutoff).

Right: the chair seen from above with the backrest folded back (sitter's left
on the left) — 4 seat + 4 backrest spots, each filled by its FSR's load
(conductance 1/R, ~linear in force), with the centre of pressure per plate;
below, one trace per mux channel (C8..C15). Which channel sits at which spot
comes from chair_map.json, built here: press m (or click a spot) and press each
sensor when asked. While a spot is waiting you can also click a channel's trace
to assign it by hand.

Keys (plot window focused):  c = calibrate (sends 'c' to the chair)   r = clear
                             m = map all 8 FSRs   Esc = cancel mapping
                             0..9 = posture tag (0 = transition; --log only)

With --log, every data row gets the current tag as an extra last column `tag`
(it starts at 0, so anything not tagged on purpose is dropped as transition);
the tag names from tags.json go in as a `# tag map` line. Without --log the tag
badge and keys are hidden. The UI text is PT-BR (shown at a science fair).

    .venv/bin/python analysis/live_posture.py
    .venv/bin/python analysis/live_posture.py --port /dev/ttyACM0 --log data/<name>.csv

Needs pyserial + PyQt6 in the venv (see requirements.txt).
"""

from __future__ import annotations

import argparse
import json
import threading
import warnings
import time
from collections import deque
from pathlib import Path

import matplotlib

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import serial  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, PowerNorm  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch  # noqa: E402

import captures as C  # noqa: E402
import chair_map as M  # noqa: E402
import fusion as F  # noqa: E402

CHAIR_PORT = "/dev/serial/by-id/usb-Seeed_XIAO_nRF52840_Sense_2C7FE46C8E63BDA0-if00"
ALERT_DEG, CLEAR_DEG = 10.0, 7.0            # keep in sync with firmware/libraries/PostureLink
# Chair CSV columns before the FSRs. Taken from the header line the firmware
# sends; until one is seen, the v1 layout (firmware before the body's report).
FIELDS = ["t_ms", "cal", "dev", "alert", "led", "dpitch", "droll", "pitch", "roll", "seq"]
CTL_NONE, CTL_CHAIR, CTL_FALLBACK = 0, 1, 2  # bctl; keep in sync with PostureLink.h
AGE_MAX = 254                                # bage: 254 = 254 samples or more
SEQ_MAX_GAP = 64
VBAT_CUTOFF_MV, VBAT_LOW_MV = 3300, 3400     # body cutoff (PostureLink/Battery.h); "low" below 3.4 V
R_FIXED, ADC_MAX, OPEN_RAW = 3300.0, 4095, 8  # chair: 3.3 kOhm divider, 12-bit ADC; <8 = open

# status colours (state, not identity) — always paired with a text label
C_OK, C_OVER, C_ALERT = "#444444", "#e69f00", "#d62728"

# chair panel: ink/chrome + one-hue sequential ramp for load (steps 100..700)
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
SURFACE, PLATE, PLATE_EC, OPEN_FC = "#fcfcfb", "#f4f3f0", "#c3c2b7", "#e8e7e2"
GRID = "#e1e0d9"
LOAD_CMAP = LinearSegmentedColormap.from_list("load", [
    "#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"])
G_MAX = 6250.0                               # µS = 160 Ω, the FSR402 floor at ~2 kg
LOAD_NORM = PowerNorm(0.5, 0, G_MAX)         # sqrt: light touches stay visible
R_TICKS = [10_000, 2_000, 1_000, 500, 250]    # Ω, on the load scale
TRACE_S = 10.0
TAGS_PATH = Path(__file__).with_name("tags.json")   # {"1": "upright", ...}; 0 is always transition


def load_tags(path: Path) -> dict[int, str]:
    """Tag -> posture name; 0 = transition, missing names = undefined."""
    names = {int(k): str(v) for k, v in json.loads(path.read_text()).items()} if path.exists() else {}
    return {**names, 0: "transition"}


def volts(mv: float) -> str:
    """'3,90 V' (PT-BR decimal comma)."""
    return f"{mv / 1000:.2f} V".replace(".", ",")


def tag_label(tags: dict[int, str], tag: int) -> str:
    """Badge text for a tag: 'transição' for 0, else the tags.json name."""
    return "transição" if tag == 0 else tags.get(tag, "(indefinida)").replace("_", " ")


def fmt_ohms(r: float) -> str:
    if r >= 1e6:
        return f"{r / 1e6:.1f} MΩ"
    if r >= 1e3:
        return f"{r / 1e3:.0f} kΩ" if r >= 1e4 else f"{r / 1e3:.1f} kΩ"
    return f"{r:.0f} Ω"


class ChairReader(threading.Thread):
    """Background serial reader; reconnects if the board resets."""

    def __init__(self, port: str, log: Path | None, trail_n: int):
        super().__init__(daemon=True)
        self.port, self.ser = port, None
        self.log = open(log, "x") if log else None   # never append to an old run
        self.lock = threading.Lock()
        self.rows: deque[dict] = deque(maxlen=trail_n)
        self.fsr: deque[tuple[float, dict[int, float]]] = deque(maxlen=int(TRACE_S * 20) + 20)
        self.events: deque[str] = deque(maxlen=3)
        self.link = ""
        self.raw = False                              # legacy default; # unit=raw selects ADC counts
        self.fields = list(FIELDS)                    # data columns before c8..c15
        self.connected = False
        self.body_up = False                          # latest data row came from the body
        self.tag = 0                                  # posture tag appended to logged rows

    def send(self, ch: str):
        if self.ser:
            try:
                self.ser.write(ch.encode())
            except serial.SerialException:
                pass

    def note(self, line: str):
        """Write a '#' comment into the log (e.g. the FSR map in use)."""
        if self.log:
            self.log.write(f"# {line}\n")
            self.log.flush()

    def _open_port(self):
        # pyserial flushes input during open(). Keep DTR low until that is
        # done, then let the firmware see a fresh connection and send headers.
        port = serial.Serial(port=None, baudrate=115200, timeout=0.5)
        port.dtr = False
        port.port = self.port
        try:
            port.open()
            time.sleep(0.05)                         # let loop() observe DTR low, including on reconnect
            port.reset_input_buffer()               # discard late bytes from the previous connection
            port.dtr = True
        except (serial.SerialException, OSError):
            port.close()
            raise
        return port

    def run(self):
        while True:
            try:
                with self._open_port() as port:
                    self.ser = port
                    self.connected, self.body_up, self.raw = True, False, False
                    with self.lock:
                        self.rows.clear()             # a new connection: no old decisions to match
                        self.fields = list(FIELDS)
                    while True:
                        line = port.readline().decode(errors="replace").strip()
                        if line:
                            self._handle(line)
            except (serial.SerialException, OSError):
                self.connected, self.ser = False, None
                time.sleep(1.0)

    def _to_us(self, v: float) -> float:
        if not self.raw:
            return M.ohms_to_us(v)
        return 1e6 * v / (R_FIXED * (ADC_MAX - v)) if OPEN_RAW <= v < ADC_MAX else 0.0

    def _handle(self, line: str):
        if self.log:
            if line.startswith("t_ms,"):
                line_out = line + ",tag"
            elif line[:1].isdigit():
                line_out = f"{line},{self.tag}"
            else:
                line_out = line
            self.log.write(line_out + "\n")
            self.log.flush()
        with self.lock:
            if line.startswith("# rx"):
                self.link = rx_summary(line[2:])
            elif line.startswith("# unit="):
                self.raw = line.startswith("# unit=raw")
            elif line.startswith("#"):
                self.events.append(line[2:])
            elif line.startswith("t_ms,"):
                fields = header_fields(line)
                if fields != self.fields:
                    self.fields = fields
                    self.rows.clear()                 # other columns: other firmware
            else:
                parsed = parse_row(line, self.fields)
                if parsed is None:
                    return
                row, fsr = parsed
                if fsr is not None:
                    self.fsr.append((time.monotonic(),
                                     {c: self._to_us(v) for c, v in zip(M.CHANNELS, fsr)}))
                self.body_up = row["seq"] >= 0        # seq=-1: body silent, IMU fields void
                if self.body_up:
                    self.rows.append(row)

    def snapshot(self):
        with self.lock:
            return (list(self.rows), list(self.fsr), list(self.events), self.link,
                    self.body_up)


def header_fields(line: str) -> list[str]:
    """Data columns of a chair header line (those before the FSRs c8..c15)."""
    cols = line.strip().split(",")
    fsr = [f"c{c}" for c in M.CHANNELS]
    return cols[:cols.index(fsr[0])] if fsr[0] in cols else cols


def parse_row(line: str, fields: list[str]):
    """(row dict, FSR values or None) for one CSV row of a chair with these
    data columns; None if it isn't one."""
    parts = line.strip().split(",")
    if len(parts) < len(fields) or not parts[0].isdigit():
        return None
    try:
        vals = [float(v) for v in parts]
    except ValueError:
        return None
    fsr = vals[len(fields):len(fields) + len(M.CHANNELS)]
    return dict(zip(fields, vals)), (fsr if len(fsr) == len(M.CHANNELS) else None)


def decision_row(rows: list[dict], i: int):
    """The chair row whose decision rows[i]'s body report holds (seq == bseq),
    looking back only through the same body session with seq stepping
    forward; None if it isn't there or can't be told apart (no basis, age
    saturated, a gap, another boot, seq going back)."""
    r = rows[i]
    if r.get("bseq", -1) < 0 or not 0 <= r.get("bage", -1) < AGE_MAX:
        return None
    for j in range(i - 1, -1, -1):
        prev, nxt = rows[j], rows[j + 1]
        step = (int(nxt["seq"]) - int(prev["seq"])) & 0xFFFF
        if prev["boot"] != r["boot"] or not 1 <= step <= SEQ_MAX_GAP:
            return None
        if prev["seq"] == r["bseq"]:
            return prev
    return None


IMU_STATES = {0: "OK", 1: "inicializando", 2: "erro I2C", 3: "sem dados novos", 4: "vetor de gravidade inválido"}


def imu_text(row: dict) -> str:
    status = int(row.get("imu", -1))
    return "IMU ? (firmware antigo)" if status < 0 else "IMU " + IMU_STATES.get(status, "estado desconhecido")


def imu_valid(row: dict) -> bool:
    # Old logs lack health metadata, but their posture data remains readable.
    return row.get("imu", 0) == 0


def battery_text(row: dict) -> str:
    """Status-box line for the body's battery (vbat, mV)."""
    mv = int(row.get("vbat", -1))
    if mv < 0:
        return "bateria ? (firmware antigo)"
    if mv == 0:
        return "bateria — sem leitura ainda"
    low = f"  BAIXA (desliga < {volts(VBAT_CUTOFF_MV)})" if mv < VBAT_LOW_MV else ""
    return f"bateria {volts(mv)}{low}"


def cutoff_hint(rows: list[dict]) -> str:
    """Banner suffix for a silent body whose last battery reading was low."""
    mv = int(rows[-1].get("vbat", -1)) if rows else -1
    return f"\núltima bateria {volts(mv)}: desligou por bateria baixa?" if 0 < mv < VBAT_LOW_MV else ""


def body_lines(rows: list[dict]) -> list[str]:
    """Status-box lines (3) for what the body reports in the latest row: who
    decides + age, the check against the chair, the motor. Firmware v1 rows
    have no report."""
    r = rows[-1]
    if "bctl" not in r:                               # v1: only the motor flag
        return ["corpo   (firmware antigo)", "        sem relatório",
                f"motor   {'● ligado' if r.get('led') else '○ desligado'}"]
    motor = "○ desligado"
    if r["bmotor"]:
        motor = "● teste" if r["btest"] else "● alerta" if r["balert"] else "● ligado"
    ctl = int(r["bctl"])
    age = int(r["bage"])
    age_txt = "?" if age < 0 else f"{age}+" if age >= AGE_MAX else f"{age}"
    if ctl == CTL_NONE:
        return ["corpo   sem decisão da", "        cadeira ainda", f"motor   {motor}"]
    if ctl == CTL_FALLBACK:                           # its own alert: nothing to confirm
        ref = "ref = da cadeira" if r["bcalid"] == r["calid"] else "ref ≠ da cadeira"
        if not r["bcal"]:
            ref = "sem ref: sem alertas"
        return ["corpo   SOZINHO (sem enlace)", f"        {ref}", f"motor   {motor}"]
    d = decision_row(rows, len(rows) - 1)
    if not imu_valid(r):
        check = "amostra inválida"
    elif r.get("binh", 0):
        check = "proteção IMU (não conferido)" if d is None else (
            "proteção IMU (inibido)" if (r["bcal"], r["bcalid"], r["balert"]) ==
            (d["cal"], d["calid"], 0) and d["alert"] else "≠ aplicado")
    elif d is None:
        check = "não conferido"
    elif (r["bcal"], r["bcalid"], r["balert"]) == (d["cal"], d["calid"], d["alert"]):
        check = "✓ aplicado"
    else:
        check = "≠ aplicado"
    return [f"corpo   cadeira, idade {age_txt}", f"        {check}", f"motor   {motor}"]


def rx_summary(text: str) -> str:
    """Chair stats line, short for the status box: 'rx pkts=20 lost=0' plus any
    other count that isn't 0 (dup/old/rst/sdrop/edrop; dt left out). Older
    firmware ('rx pkts=N lost=M') passes through unchanged."""
    words = text.split()
    keep = [w for w in words[1:] if "=" in w and not w.startswith("dt=")
            and (w.split("=")[0] in ("pkts", "lost") or w.split("=")[1] != "0")]
    return " ".join(words[:1] + keep)


REF_POSTURES = {                              # display group -> (tags, label, label offset pt, ha)
    "curvado": ([2], "curvado p/ frente", (10, 0), "left"),
    "cabeca": ([3], "cabeça p/ frente", (10, 0), "left"),
    "escorregado": ([4], "escorregado", (-10, 0), "right"),
    "esq": ([5], "inclinado à esq.", (-10, 0), "right"),
    "dir": ([6], "inclinado à dir.", (10, 0), "left"),
    "reclinado": ([7], "reclinado", (10, 0), "left"),
    "pernas": ([8, 9], "pernas cruzadas", (10, 8), "left"),
}
REF_BAD, REF_OK = "#e07b72", "#5aa469"           # should alert / must not (fusion.BAD_TAGS)
REF_GLOB = "2026-10-21_*_run*.csv"               # the tagged multi-subject captures (data/README.md)


def load_reference_runs(pattern: str = REF_GLOB, skip: Path | None = None) -> list:
    """Captures for the overlay; best effort: an unreadable file (e.g. a log cut
    short before its first row) is skipped, never fatal for the live view."""
    runs = []
    for p in sorted(C.DATA.glob(pattern)):
        if skip is not None and p.resolve() == skip.resolve():
            continue
        try:
            with warnings.catch_warnings():         # untrusted-calibration notes
                warnings.simplefilter("ignore")
                runs.append(C.load(p))
        except (ValueError, IndexError, KeyError, OSError) as e:
            print(f"reference: skipping {p.name} ({e})")
    return runs


def reference_points(runs) -> dict[str, np.ndarray]:
    """Display group -> per-subject median (dpitch, droll), one row per subject
    that held it (all their runs pooled): settled, calibrated, valid-IMU rows
    of the tagged captures."""
    per: dict[str, list] = {k: [] for k in REF_POSTURES}
    by_subject: dict[str, list] = {}
    for run in runs:
        d = run.rows
        by_subject.setdefault(run.subject, []).append(
            d[d["settled"] & (d["cal"] == 1) & (d["imu"] == 0 if "imu" in d else True)])
    for frames in by_subject.values():
        d = pd.concat(frames)
        for key, (tags, *_) in REF_POSTURES.items():
            g = d[d["tag"].isin(tags)]
            if len(g):
                per[key].append((g["dpitch"].median(), g["droll"].median()))
    return {k: np.array(v) for k, v in per.items() if v}


def draw_reference(ax, refs: dict[str, np.ndarray], n_subjects: int):
    """Recorded postures: one small dot per subject, a big one at their median."""
    for key, pts in refs.items():
        tags, label, off, ha = REF_POSTURES[key]
        c = REF_BAD if set(tags) <= set(F.BAD_TAGS) else REF_OK
        ax.scatter(pts[:, 0], pts[:, 1], s=22, color=c, alpha=0.35, lw=0, zorder=2)
        mx, my = np.median(pts, axis=0)
        ax.scatter([mx], [my], s=150, color=c, edgecolor="white", lw=2, alpha=0.9, zorder=3)
        ax.annotate(label, (mx, my), xytext=off, textcoords="offset points", ha=ha,
                    va="center", fontsize=9, color=INK2, zorder=3)
    for c, txt in ((REF_BAD, "deve alertar"), (REF_OK, "não alerta")):
        ax.scatter([], [], s=80, color=c, edgecolor="white", label=txt)
    ax.legend(loc="lower left", fontsize=9, frameon=False,
              title=f"posturas gravadas ({n_subjects} pessoas)", title_fontsize=9)


class ChairView:
    """Chair diagram + per-channel traces + the interactive FSR mapping."""

    PLATES = {"back": (0.0, 11.0), "seat": (0.0, 0.0)}   # lower-left corner, 10 x 10
    SPOT_R = 2.0

    def __init__(self, fig, rect_chair, rect_bar, rect_traces, map_path: Path, reader):
        self.map_path, self.reader = map_path, reader
        self.map = M.load_map(map_path)
        self.session: M.MapSession | None = None
        self.last_t = 0.0
        self.flash: tuple[str, float] | None = None   # (slot, until) after an assignment

        ax = self.ax = fig.add_axes(rect_chair)
        ax.set_xlim(-1.5, 25)
        ax.set_ylim(-2, 21.2)
        ax.set_aspect("equal")
        ax.axis("off")
        names = {"back": "ENCOSTO", "seat": "ASSENTO"}
        self.spots, self.spot_txt, self.spot_val, self.cop, self.summary = {}, {}, {}, {}, {}
        for plate, (x0, y0) in self.PLATES.items():
            ax.add_patch(FancyBboxPatch((x0, y0), 10, 10, boxstyle="round,pad=0,rounding_size=1.2",
                                        fc=PLATE, ec=PLATE_EC, lw=1, zorder=0))
            ax.text(x0 + 10.8, y0 + 9.6, names[plate], fontsize=10, weight="bold",
                    color=INK, va="top")
            self.summary[plate] = ax.text(x0 + 10.8, y0 + 8.2, "", fontsize=9, color=INK2,
                                          va="top", linespacing=1.5)
            self.cop[plate], = ax.plot([], [], marker="+", ms=16, mew=2.5, color=INK, zorder=5)
        ax.text(5, -1.2, "borda da frente", ha="center", fontsize=8, color=MUTED)
        ax.text(5, 10.5, "dobra", ha="center", va="center", fontsize=7, color=MUTED)
        ax.text(-0.9, 10.5, "E", ha="center", va="center", fontsize=9, color=MUTED, weight="bold")
        ax.text(10.9, 10.5, "D", ha="center", va="center", fontsize=9, color=MUTED, weight="bold")
        for slot in M.SLOTS:
            cx, cy = self._xy(slot)
            self.spots[slot] = ax.add_patch(Circle((cx, cy), self.SPOT_R, zorder=2))
            self.spot_txt[slot] = ax.text(cx, cy + 0.45, "", ha="center", va="center",
                                          fontsize=10, weight="bold", zorder=3)
            self.spot_val[slot] = ax.text(cx, cy - 0.6, "", ha="center", va="center",
                                          fontsize=7.5, zorder=3)
        x0, y0, w, h = rect_chair
        self.title = fig.text(x0, 0.985, "", fontsize=10, color=INK, va="top", ha="left")
        self.prompt = fig.text(x0 + w / 2, y0 + h + 0.005, "", ha="center", va="bottom",
                               fontsize=11, color=INK,
                              weight="bold", zorder=6,
                              bbox=dict(boxstyle="round,pad=0.5", fc="white", ec=INK, lw=1.2))

        cb = fig.add_axes(rect_bar)
        grad = np.linspace(0, 1, 256)[None, :]
        cb.imshow(grad, aspect="auto", cmap=LOAD_CMAP, extent=(0, 1, 0, 1))
        cb.set_yticks([])
        cb.set_xticks([float(LOAD_NORM(1e6 / r)) for r in R_TICKS],
                      [fmt_ohms(r) for r in R_TICKS], fontsize=7, color=MUTED)
        cb.tick_params(length=2, color=PLATE_EC)
        for s in cb.spines.values():
            s.set_visible(False)
        cb.set_title("carga por FSR  (R, ~1/força)   cinza = solto", fontsize=8, color=INK2,
                     loc="left")

        x0, y0, w, h = rect_traces
        self.trace_ax, self.trace_ln, self.trace_ti = {}, {}, {}
        for i, ch in enumerate(M.CHANNELS):
            r, c = divmod(i, 4)
            ta = fig.add_axes((x0 + c * w / 4 + 0.006, y0 + (1 - r) * h / 2 + 0.012,
                               w / 4 - 0.012, h / 2 - 0.045))
            ta.set_xlim(-TRACE_S, 0)
            ta.set_ylim(0, 1.02)
            ta.set_yticks([float(LOAD_NORM(1e6 / rr)) for rr in (10_000, 1_000, 250)],
                          ["10k", "1k", "250"] if c == 0 else [], fontsize=6, color=MUTED)
            ta.set_xticks([-10, -5, 0], ["−10 s", "−5", "0"] if r == 1 else [],
                          fontsize=6, color=MUTED)
            ta.tick_params(length=0)
            ta.grid(color=GRID, lw=0.6)
            for s in ta.spines.values():
                s.set_color(GRID)
            ta.set_facecolor(SURFACE)
            self.trace_ln[ch], = ta.plot([], [], color="#2a78d6", lw=1.6)
            self.trace_ti[ch] = ta.set_title("", fontsize=8, loc="left", pad=2)
            self.trace_ax[ch] = ta
        self._log_map()

    def _xy(self, slot: str):
        plate, _, sx, sy = M.SLOTS[slot]
        x0, y0 = self.PLATES[plate]
        return x0 + 2.5 + 5 * sx, y0 + 2.5 + 5 * sy

    # --- mapping ---------------------------------------------------------
    def start(self, slots: list[str]):
        self.session = M.MapSession(slots)

    def cancel(self):
        self.session = None

    def _assigned(self, slot: str, ch: int):
        self.map = M.assign(self.map, slot, ch)
        M.save_map(self.map, self.map_path)
        self.flash = (slot, time.monotonic() + 0.8)
        self._log_map()

    def _log_map(self):
        inv = {c: s for s, c in self.map.items()}
        self.reader.note("fsr map " + " ".join(
            f"c{c}={inv.get(c, 'unmapped')}" for c in M.CHANNELS))

    def on_click(self, ev):
        if ev.inaxes is self.ax and ev.xdata is not None:
            for slot in M.SLOTS:
                cx, cy = self._xy(slot)
                if np.hypot(ev.xdata - cx, ev.ydata - cy) <= self.SPOT_R * 1.2:
                    self.start([slot])
                    return
        if self.session and self.session.state in ("settle", "press"):
            for ch, ta in self.trace_ax.items():
                if ev.inaxes is ta and ch not in self.session.taken:
                    slot, ch = self.session.pick(ch, time.monotonic())
                    self._assigned(slot, ch)
                    return

    # --- drawing ---------------------------------------------------------
    def update(self, fsr):
        now = time.monotonic()
        if self.session:
            for t, g in fsr:
                if t > self.last_t:
                    hit = self.session.feed(t, g)
                    if hit:
                        self._assigned(*hit)
            if self.session.done:
                self.session = None
        if fsr:
            self.last_t = fsr[-1][0]
        g = fsr[-1][1] if fsr else {}
        live = bool(fsr) and now - fsr[-1][0] < 1.0

        waiting = self.session.slot if self.session else None
        cand = self.session.candidate() if self.session else None
        flash = self.flash[0] if self.flash and now < self.flash[1] else None
        for slot, spot in self.spots.items():
            ch = self.map.get(slot)
            txt, val = self.spot_txt[slot], self.spot_val[slot]
            if ch is None or not live:
                spot.set(fc="none" if ch is None else OPEN_FC, ec=MUTED, lw=1.2,
                         ls=":" if ch is None else "-")
                txt.set(text="?" if ch is None else f"C{ch}", color=MUTED)
                val.set(text="clique" if ch is None else "—", color=MUTED)
            else:
                us = g.get(ch, 0.0)
                lvl = float(LOAD_NORM(us))
                spot.set(fc=LOAD_CMAP(lvl) if us > 0 else OPEN_FC, ec=SURFACE, lw=2, ls="-")
                ink = "white" if lvl > 0.55 else INK
                txt.set(text=f"C{ch}", color=ink)
                val.set(text=fmt_ohms(1e6 / us) if us > 0 else "solto", color=ink)
            if slot == waiting:
                spot.set(ec=INK, lw=3, ls="-")
                txt.set_color(INK if ch is None else txt.get_color())
                val.set_text({"settle": "sem mãos", "press": "aperte",
                              "release": "solte"}[self.session.state])
            elif slot == flash:
                spot.set(ec="#0ca30c", lw=3.5, ls="-")

        bal = {p: M.plate_balance(self.map, g if live else {}, p) for p in self.PLATES}
        chair_total = sum(b[2] for b in bal.values() if b)
        for plate in self.PLATES:
            self._plate_summary(plate, bal[plate], chair_total)

        title = "FSRs da cadeira — vista de cima, encosto rebatido (esquerda de quem senta = esquerda)"
        if not live:
            title += "\nsem dados dos FSRs" + ("" if fsr else " ainda (firmware antigo da cadeira?)")
        self.title.set_text(title)
        if self.session:
            hint = ("ou clique no gráfico dele abaixo · " if self.session.state == "press" else "")
            self.prompt.set_text(f"{self.session.prompt}\n{hint}Esc cancela")
        elif len(self.map) < len(M.SLOTS):
            self.prompt.set_text(f"{len(self.map)}/8 FSRs mapeados — aperte m para mapear")
        else:
            self.prompt.set_text("")
        self.prompt.set_visible(bool(self.prompt.get_text()))

        inv = {c: s for s, c in self.map.items()}
        for ch in M.CHANNELS:
            pts = [(t - now, float(LOAD_NORM(gg.get(ch, 0.0)))) for t, gg in fsr]
            self.trace_ln[ch].set_data([p[0] for p in pts], [p[1] for p in pts])
            where = M.slot_name(inv[ch]) if ch in inv else "não mapeado"
            hot = ch == cand
            self.trace_ti[ch].set(text=f"C{ch} · {where}" + ("  ◀ apertado" if hot else ""),
                                  color=INK if hot or ch in inv else MUTED,
                                  weight="bold" if hot else "normal")
            self.trace_ax[ch].set_facecolor("#eef4fc" if hot else SURFACE)

    def _plate_summary(self, plate: str, bal, chair_total: float):
        mapped = [s for s in self.map if M.SLOTS[s][0] == plate]
        txt, cop = self.summary[plate], self.cop[plate]
        cop.set_data([], [])
        if len(mapped) < 4:
            txt.set_text(f"{len(mapped)}/4 mapeados")
            return
        if bal is None:
            txt.set_text("sem carga")
            return
        x, y, total = bal
        x0, y0 = self.PLATES[plate]
        cop.set_data([x0 + 2.5 + 5 * x], [y0 + 2.5 + 5 * y])
        lo, hi = ("frente", "trás") if plate == "seat" else ("baixo", "cima")
        txt.set_text(f"{100 * total / chair_total:3.0f} % da carga da cadeira\n"
                     f"E {100 * (1 - x):3.0f} %  ·  D {100 * x:3.0f} %\n"
                     f"{lo} {100 * (1 - y):3.0f} %  ·  {hi} {100 * y:3.0f} %\n"
                     "+ = centro de pressão")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default=CHAIR_PORT)
    ap.add_argument("--map", type=Path, default=M.MAP_PATH,
                    help="FSR channel -> chair spot map (read + written)")
    ap.add_argument("--log", type=Path, help="also save every serial line here (new file; refuses an existing one)")
    ap.add_argument("--trail", type=float, default=3.0, help="trail length, s")
    ap.add_argument("--tags", type=Path, default=TAGS_PATH,
                    help="posture tag names (JSON, {\"1\": \"name\", ...})")
    ap.add_argument("--no-ref", action="store_true",
                    help="don't draw the recorded postures (data/*_run*.csv) on the IMU plot")
    args = ap.parse_args()
    if args.log and args.log.exists():
        ap.error(f"{args.log} already exists; pick a new name (e.g. the next _runN)")

    tags = load_tags(args.tags)
    ref_runs = [] if args.no_ref else load_reference_runs(skip=args.log)   # before the log exists
    reader = ChairReader(args.port, args.log, trail_n=max(2, int(args.trail * 20)))
    reader.note("tag map " + " ".join(f"{k}={v.replace(' ', '_')}" for k, v in sorted(tags.items())))
    reader.start()

    for k in ("keymap.back", "keymap.home"):          # free 'c' and 'r'
        plt.rcParams[k] = [x for x in plt.rcParams[k] if x not in ("c", "r")]

    fig = plt.figure(figsize=(17, 9.5))
    fig.canvas.manager.set_window_title("Postura — ao vivo (IMU do corpo + FSRs da cadeira)")
    ax = fig.add_axes((0.04, 0.08, 0.39, 0.84))
    chair = ChairView(fig, (0.575, 0.37, 0.41, 0.50), (0.59, 0.335, 0.2, 0.013),
                      (0.575, 0.03, 0.41, 0.27), args.map, reader)
    ax.add_patch(Circle((0, 0), ALERT_DEG, fill=False, color="#888", lw=1.5))
    ax.add_patch(Circle((0, 0), CLEAR_DEG, fill=False, color="#888", lw=1, ls=":"))
    ax.annotate(f"alerta {ALERT_DEG:.0f}°", (ALERT_DEG * 0.71, ALERT_DEG * 0.71),
                xytext=(4, 4), textcoords="offset points", fontsize=8, color="#666")
    ax.axhline(0, color="#ddd", lw=0.8, zorder=0)
    ax.axvline(0, color="#ddd", lw=0.8, zorder=0)
    ax.set_xlim(-40, 40)
    ax.set_ylim(-55, 30)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    # neck mount: dpitch = sideways lean, droll = forward/back (data/ captures)
    ax.set_xlabel("dpitch [°]   (− esquerda · direita +)")
    ax.set_ylabel("droll [°]   (− frente · trás +)")
    ax.set_title("Desvio da postura calibrada (centro = ereto)", fontsize=10)
    if ref_runs:
        draw_reference(ax, reference_points(ref_runs), len({r.subject for r in ref_runs}))

    trail, = ax.plot([], [], color="#333", lw=2, alpha=0.5, zorder=4)
    dot = ax.scatter([0], [0], s=260, color=C_OK, edgecolor="white", lw=2.5, zorder=6)
    status = fig.text(0.445, 0.92, "", ha="left", va="top", family="monospace", fontsize=9,
                      bbox=dict(boxstyle="round", fc="white", ec="#ccc"))
    banner = ax.text(0.5, 0.5, "", transform=ax.transAxes, ha="center", va="center",
                     fontsize=13, color="#333",
                     bbox=dict(boxstyle="round", fc="white", ec="#ccc", alpha=0.9))
    tag_badge = ax.text(0.02, 0.98, "", transform=ax.transAxes, ha="left", va="top",
                        fontsize=16, weight="bold", zorder=7, visible=bool(args.log),
                        bbox=dict(boxstyle="round,pad=0.5", lw=1.5))
    tag_set_at = [0.0]                                # monotonic time of the last tag key

    def on_key(ev):
        if ev.key in ("c", "r"):
            reader.send(ev.key)
        elif ev.key == "m":
            chair.start(M.WIZARD_ORDER)
        elif ev.key == "escape":
            chair.cancel()
        elif args.log and ev.key in tuple("0123456789"):     # tags only matter in a log
            reader.tag = int(ev.key)
            tag_set_at[0] = time.monotonic()
    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("button_press_event", chair.on_click)

    keys = ("\n\nteclas\n· c calibrar\n· r limpar calibração\n· m mapear FSRs"
            "\n· Esc cancelar mapeamento" + ("\n· 0-9 tag (0 = transição)" if args.log else ""))

    def update(_):
        rows, fsr, events, link, body_up = reader.snapshot()
        body_up = body_up and bool(rows)
        chair.update(fsr)
        version = next((w for w in link.split() if w.startswith("ver=")), "")
        bad = next((w for w in link.split() if w.startswith("bad=")), "")
        if not reader.connected:
            banner.set_text(f"aguardando {Path(args.port).name} …")
        elif not body_up and version:
            banner.set_text(f"PROTOCOLO DO CORPO INCOMPATÍVEL ({version})\noutra versão de firmware? "
                            "grave as duas placas")
        elif not body_up and bad:
            banner.set_text(f"PACOTES DO CORPO REJEITADOS ({bad})\npayloads malformados")
        elif not body_up:
            banner.set_text("CORPO EM SILÊNCIO — sem dados da IMU\n(rastro = últimas amostras)" + cutoff_hint(rows)
                            if rows else "conectado, aguardando o corpo …")
        elif not imu_valid(rows[-1]):
            banner.set_text(imu_text(rows[-1]) + "\namostra de postura indisponível")
        elif not rows[-1]["cal"]:
            banner.set_text("NÃO CALIBRADO\nsente-se ereto e aperte o botão da cadeira "
                            "(ou 'c' aqui)")
        else:
            banner.set_text("")
        banner.set_visible(bool(banner.get_text()))

        cal_rows = [r for r in rows if r["cal"] and imu_valid(r)] if rows and imu_valid(rows[-1]) else []
        if cal_rows:
            x = np.array([r["dpitch"] for r in cal_rows])
            y = np.array([r["droll"] for r in cal_rows])
            trail.set_data(x, y)
            trail.set_alpha(0.5 if body_up else 0.15)
            dot.set_offsets([[x[-1], y[-1]]])
            dot.set_visible(body_up)
        else:
            trail.set_data([], [])
            dot.set_visible(False)

        tag = ""
        if args.log:
            name = tag_label(tags, reader.tag)
            flash = time.monotonic() - tag_set_at[0] < 0.6     # confirms the key press
            tag_badge.set_text(f"TAG {reader.tag} · {name}")
            tag_badge.set_color(INK if reader.tag == 0 else "white")
            tag_badge.get_bbox_patch().set(
                fc=OPEN_FC if reader.tag == 0 else "#2a78d6",
                ec="#0ca30c" if flash else PLATE_EC, lw=4 if flash else 1.5)
            tag = f"tag     {reader.tag} · {name}\n"
        tail = (f"{tag}enlace  {link.removeprefix('rx ') or '—'}\n"
                + "\núltimos eventos\n" + "\n".join(f"· {e[:26]}" for e in events) + keys)
        if not rows or not body_up:
            status.set_text("IMU —\ndesvio  —\ncadeira — corpo em silêncio\ncorpo   —\n\n"
                            "motor   —\nbateria —\n" + tail)
        else:
            r = rows[-1]
            if not imu_valid(r):
                colour, state = C_OVER, "— amostra inválida"
            elif r["alert"]:
                colour, state = C_ALERT, "▲ ALERTA"
            elif r["cal"] and r["dev"] > ALERT_DEG:
                colour, state = C_OVER, "● acima do limiar"
            else:
                colour, state = C_OK, ("✓ ok" if "imu" in r else "ok (IMU ?)") if r["cal"] else "— não calibrado"
            dot.set_color(colour)
            dot.set_edgecolor("white")
            status.set_text(
                f"{imu_text(r)}\ndesvio  {r['dev']:5.1f}°\n".replace(".", ",") +
                f"cadeira {state}\n" + "\n".join(body_lines(rows)) + "\n" + battery_text(r) + "\n" + tail)

    _anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)  # noqa: F841
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
