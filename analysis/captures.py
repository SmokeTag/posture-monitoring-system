"""Loader for the full-system captures (data/YYYY-MM-DD_<subject>_run<N>.csv).

One file = the chair unit's CSV (firmware/chair/chair.ino) as logged by
live_posture.py --log: '#' lines are comments/events, interleaved with the rows
in stream order; the last column is the posture tag (0 = transition). Newer
firmware adds a `reason` column (the fusion rule's reason code) after c15 and
the FSR baseline (fsr=c8..c15) to the "# calibrated" event; both are optional.

    run = load("data/2026-10-21_Rafael-Herleng_run1.csv")
    run.rows                  # DataFrame, one row per sample; + t (s), seg, settled
    run.fsr()                 # DataFrame of the 8 FSRs by spot name (raw ADC)
    run.cals                  # calibrations: calid, IMU reference, capture window
    run.fsr_baseline(cal)     # spot -> mean raw ADC over that capture window

Calibration timing (chair.ino onBodySample): "# calibrating" starts a capture
of CAL_SAMPLES = 20 body samples; the "# calibrated" event is printed while the
20th is processed, before its row, which already carries the new calid. So the
capture window is the 19 rows before the event + the first row after it.
"""

from __future__ import annotations

import re
import warnings
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
CAL_SAMPLES = 20                              # chair.ino: 1 s at 20 Hz
SETTLE_S = 2.0                                # dropped after every tag change
SPOTS = ["seat_front_left", "seat_front_right", "seat_rear_left", "seat_rear_right",
         "back_lower_left", "back_lower_right", "back_upper_left", "back_upper_right"]

_NAME = re.compile(r"(\d{4}-\d{2}-\d{2})_(.+)_run(\d+)\.csv$")
_CALIBRATED = re.compile(r"# calibrated upright: pitch=(\S+) roll=(\S+) calid=(\d+)(?: fsr=(\S+))?")


@dataclass
class Cal:
    calid: int
    row: int                                  # first row carrying this calid
    pitch: float = np.nan                     # IMU reference (deg); nan if not logged
    roll: float = np.nan
    start: int = 0                            # capture window rows [start, stop)
    stop: int = 0
    in_file: bool = True                      # False: captured before logging started
    fsr: dict[str, float] | None = None       # FSR baseline logged by the chair (c8.. -> mean), if any
    trusted: bool = True                      # False: logged window incomplete (dropped rows/events)


@dataclass
class Run:
    path: Path
    subject: str
    run: int
    rows: pd.DataFrame
    events: list[tuple[int, str]]             # (rows before the line, text)
    fsr_map: dict[str, str]                   # spot -> column ("c8".."c15")
    tags: dict[int, str]
    cals: list[Cal] = field(default_factory=list)

    @property
    def name(self) -> str:
        return f"{self.subject}/run{self.run}"

    def fsr(self) -> pd.DataFrame:
        return pd.DataFrame({s: self.rows[c].to_numpy(float) for s, c in self.fsr_map.items()},
                            index=self.rows.index)[SPOTS]

    def cal(self, calid: int) -> Cal | None:
        return next((c for c in self.cals if c.calid == calid), None)

    def fsr_baseline(self, cal: Cal) -> dict[str, float]:
        """Spot -> mean raw ADC over the capture window (the chair's own, if logged)."""
        if cal.fsr:
            return {s: cal.fsr[self.fsr_map[s]] for s in SPOTS}
        f = self.fsr().iloc[cal.start:cal.stop]
        return {s: float(f[s].mean()) for s in SPOTS}


def parse_map(line: str) -> dict[str, str]:
    """'# fsr map c8=seat_front_left ...' -> {'seat_front_left': 'c8', ...}"""
    pairs = (p.split("=", 1) for p in line.split()[3:] if "=" in p)
    return {spot: ch for ch, spot in pairs}


def parse_tags(line: str) -> dict[int, str]:
    """'# tag map 0=transition 1=ereto ...' -> {0: 'transition', 1: 'ereto', ...}"""
    pairs = (p.split("=", 1) for p in line.split()[3:] if "=" in p)
    return {int(k): v for k, v in pairs}


def load(path: str | Path) -> Run:
    path = Path(path)
    m = _NAME.search(path.name)
    subject, run_n = (m.group(2), int(m.group(3))) if m else (path.stem, 0)
    cols, data, events = None, [], []
    fsr_map, tags = {}, {}
    for line in path.read_text().splitlines():
        if not line:
            continue
        if line.startswith("#"):
            events.append((len(data), line))
            if line.startswith("# fsr map"):
                fsr_map = parse_map(line)
            elif line.startswith("# tag map"):
                tags = parse_tags(line)
            continue
        if line.startswith("t_ms"):
            cols = line.split(",")
            continue
        v = line.split(",")
        if cols and len(v) == len(cols):
            data.append(v)
    if not cols:
        raise ValueError(f"{path}: no column line")
    rows = pd.DataFrame(data, columns=cols).apply(pd.to_numeric, errors="coerce")
    rows["t"] = (rows["t_ms"] - rows["t_ms"].iloc[0]) / 1000.0
    rows["seg"] = (rows["tag"] != rows["tag"].shift()).cumsum() - 1
    rows["settled"] = settled(rows)
    r = Run(path, subject, run_n, rows, events, fsr_map, tags)
    r.cals = find_cals(rows, events)
    return r


def find_cals(rows: pd.DataFrame, events: list[tuple[int, str]]) -> list[Cal]:
    """Calibrations with their capture windows. A calid already in effect when
    logging started (no event) gets its first CAL_SAMPLES rows, in_file=False."""
    cals = []
    calid = rows["calid"].to_numpy()
    for n, text in events:
        m = _CALIBRATED.match(text)
        if not m or n >= len(rows):
            continue
        cid = int(m.group(3))
        if calid[n] != cid:                    # the row after the event carries it
            continue
        logged = None
        if m.group(4):                          # chair firmware since 2026-10-22: c8..c15 means
            vals = [float(v) for v in m.group(4).split(",")]
            logged = {f"c{8 + i}": v for i, v in enumerate(vals)}
        cal = Cal(cid, n, float(m.group(1)), float(m.group(2)),
                  max(0, n - (CAL_SAMPLES - 1)), n + 1, fsr=logged)
        if logged is None:
            problem = window_problem(rows, events, cal)
            if problem:
                cal.trusted = False
                warnings.warn(f"calid={cid} at row {n}: capture window not reconstructable "
                              f"({problem}); calibration untrusted", stacklevel=3)
        cals.append(cal)
    known = {c.calid for c in cals}
    first = rows.loc[rows["cal"] == 1, "calid"]
    if len(first) and int(first.iloc[0]) not in known:
        i = int(first.index[0])
        cals.insert(0, Cal(int(first.iloc[0]), i, start=i, stop=i + CAL_SAMPLES, in_file=False))
    return cals


def window_problem(rows: pd.DataFrame, events: list[tuple[int, str]], cal: Cal) -> str:
    """Why rows [start, stop) are not exactly the CAL_SAMPLES capture samples, or ''.
    The chair drops whole rows/events when the USB host lags (sdrop/edrop), so
    the 19 rows before '# calibrated' may reach back into other postures."""
    w = rows.iloc[cal.start:cal.stop]
    if len(w) != CAL_SAMPLES:
        return f"{len(w)} rows"
    if not any(n == cal.start and t.startswith("# calibrating") for n, t in events):
        return "'# calibrating' not right before the window"
    if any(cal.start < n < cal.row and t.startswith("# calibrat") for n, t in events):
        return "another calibration event inside the window"
    seq, boot = w["seq"].to_numpy(np.int64), w["boot"].to_numpy()
    if (boot != boot[0]).any() or ((np.diff(seq) % 65536) != 1).any():
        return "body samples missing (seq gap) or another body session"
    if "imu" in w and (w["imu"] != 0).any():
        return "invalid IMU sample"
    calid = w["calid"].to_numpy()
    if (calid[:-1] != calid[0]).any() or calid[0] == cal.calid:
        return "calid changes inside the window"
    return ""


def settled(rows: pd.DataFrame, skip_s: float = SETTLE_S) -> np.ndarray:
    """Tagged (tag != 0) and at least skip_s after the segment's first row."""
    t0 = rows.groupby("seg")["t"].transform("first")
    return ((rows["tag"] != 0) & (rows["t"] - t0 >= skip_s)).to_numpy()


def segments(rows: pd.DataFrame) -> pd.DataFrame:
    """One line per tagged segment: seg, tag, first/last row, t0/t1 (s)."""
    g = rows[rows["tag"] != 0].groupby("seg")
    out = pd.DataFrame({"tag": g["tag"].first(), "first": g.apply(lambda d: d.index[0]),
                        "last": g.apply(lambda d: d.index[-1]),
                        "t0": g["t"].first(), "t1": g["t"].last()})
    return out.reset_index()


def runs(data: Path = DATA, pattern: str = "????-??-??_*_run*.csv") -> list[Run]:
    return [load(p) for p in sorted(data.glob(pattern))]
