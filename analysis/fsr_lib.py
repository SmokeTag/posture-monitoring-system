"""Shared loader + helpers for the FSR402 characterization captures (Campaign B).

The CSVs are produced by firmware/uno_fsr_test/uno_fsr_test.ino: an Arduino Uno
reads one FSR402 through a voltage divider and dumps serial at ~10 Hz in CSV
mode. Columns:

    t_ms,load_kg,raw,ohms
      t_ms      millis() timestamp since boot
      load_kg   last typed scale reading (the applied load stamp). **Units caveat:**
                the precision-balance protocol stamps GRAMS into this column
                (50,100,...,2000); divide by 1000 for kg. ohms is unit-agnostic.
      raw       0..1023 ADC count (10-bit, Uno)
      ohms      computed FSR resistance; -1 means open / no force

Parsing gotchas this module handles so callers don't have to:
  * The operator usually opens `arduino-cli monitor | tee` BEFORE pressing 'l',
    so the file starts with human-readable `raw\\tV\\tohms` lines (tab-separated,
    not CSV) — these are dropped (t_ms not numeric).
  * Serial races occasionally concatenate a '#' prose line onto a data row, e.g.
    `927592,-1.00,# FSR402 characterization...`. `comment='#'` truncates at the
    '#', leaving a short row that coerces cleanly (raw/ohms -> NaN, dropped).
  * Each load level is typed by hand, so the same nominal level is stamped with
    slightly different integers across an up/down sweep or repeats (399 vs 400,
    1233/1234/1235, 1622 vs 1623). Grouping by the *exact* load_kg fragments the
    curve; `add_canonical()` / `cluster_loads()` merge that stamp jitter.

Resistance (ohms) is the board-independent quantity, so these helpers apply
unchanged to a future 3.3 V XIAO capture.
"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import pandas as pd

COLS = ["t_ms", "load_kg", "raw", "ohms"]

# Repo root = parent of the analysis/ directory this file lives in.
REPO_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = REPO_ROOT / "data"
FIG_DIR = Path(__file__).resolve().parent / "figures"

G = 9.80665          # kg -> N
ADC_MAX = 1023       # Uno 10-bit ADC; raw near this = railed / no resolution

# Canonical load ladder (grams) used to align levels ACROSS sensors. Fine enough
# to keep ~1.4 vs ~1.6 kg distinct, coarse enough to absorb hand-stamp jitter.
LADDER_G = [50, 100, 200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800, 2000]


def latest_csv(pattern: str = "*fsr*.csv") -> Path:
    """Newest matching capture under data/. NOTE: same-day FSR files do not sort
    by capture time (the sensor/kind tokens dominate), so prefer an explicit
    --csv for a specific run; this is only a convenience default."""
    files = sorted(DATA_DIR.glob(pattern))
    return files[-1] if files else DATA_DIR / "<no fsr capture yet>.csv"


def load(path: str | Path) -> pd.DataFrame:
    """Parse an FSR capture into a clean, fully-numeric, file-ordered frame.

    Adds `t_s` and `t_min` (relative to the first row) for the creep plots.
    Robust to the serial-interleaving artifacts documented in the module header.
    """
    df = pd.read_csv(path, comment="#", header=None, names=COLS,
                     engine="python", on_bad_lines="skip")
    df = df[pd.to_numeric(df["t_ms"], errors="coerce").notna()].copy()
    for c in COLS:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=["t_ms"]).reset_index(drop=True)
    if len(df):
        df["t_s"] = (df["t_ms"] - df["t_ms"].min()) / 1000.0
        df["t_min"] = df["t_s"] / 60.0
    else:
        df["t_s"] = df["t_min"] = pd.Series(dtype=float)
    return df


def loaded(df: pd.DataFrame) -> pd.DataFrame:
    """Rows with an actual load applied (ohms > 0 and a positive load stamp)."""
    return df[(df["ohms"] > 0) & (df["load_kg"] > 0)].copy()


def cluster_loads(values, rel_tol: float = 0.05) -> dict[float, int]:
    """Merge hand-stamp jitter: map each raw load value to a canonical level.

    1-D single-link clustering over the sorted unique values — consecutive values
    within `rel_tol` (relative) join the same cluster; each cluster collapses to
    the rounded median of its members. Returns {raw_load: canonical_int}.
    """
    uniq = sorted({float(v) for v in values if v > 0})
    if not uniq:
        return {}
    clusters: list[list[float]] = [[uniq[0]]]
    for v in uniq[1:]:
        if v <= clusters[-1][-1] * (1 + rel_tol):
            clusters[-1].append(v)
        else:
            clusters.append([v])
    mapping: dict[float, int] = {}
    for cl in clusters:
        canon = int(round(float(np.median(cl))))
        for v in cl:
            mapping[v] = canon
    return mapping


def add_canonical(df: pd.DataFrame, rel_tol: float = 0.05) -> pd.DataFrame:
    """Return a copy of loaded rows with a `load_canon` column (jitter merged)."""
    dfl = loaded(df)
    mapping = cluster_loads(dfl["load_kg"], rel_tol=rel_tol)
    dfl["load_canon"] = dfl["load_kg"].map(mapping)
    return dfl


def snap_to_ladder(value: float, ladder=LADDER_G, rel_tol: float = 0.18):
    """Snap a (canonical) load to the nearest shared-ladder rung, or None if no
    rung is within `rel_tol` — so cross-sensor levels line up on common x-values."""
    best, bestd = None, None
    for rung in ladder:
        d = abs(value - rung) / rung
        if d <= rel_tol and (bestd is None or d < bestd):
            best, bestd = rung, d
    return best


def per_level(dfl: pd.DataFrame, col: str = "load_canon") -> pd.DataFrame:
    """Median/spread of resistance per load level (loaded rows only).

    Columns: <col>, median, std, count, q25, q75, raw_med (median ADC count).
    """
    g = dfl.groupby(col).agg(
        median=("ohms", "median"),
        std=("ohms", "std"),
        count=("ohms", "count"),
        q25=("ohms", lambda s: s.quantile(0.25)),
        q75=("ohms", lambda s: s.quantile(0.75)),
        raw_med=("raw", "median"),
    ).reset_index().sort_values(col)
    return g.reset_index(drop=True)


def split_direction(dfl: pd.DataFrame) -> pd.DataFrame:
    """Label each loaded row 'up' or 'down' by the loading direction.

    A sweep goes light -> heavy -> light, so the time of peak load splits the
    file: rows at/ before it are the up-sweep, rows after are the down-sweep.
    Returns a copy with a `direction` column.
    """
    if dfl.empty:
        return dfl.assign(direction=pd.Series(dtype=object))
    dfl = dfl.sort_values("t_ms")
    peak_t = dfl.loc[dfl["load_kg"].idxmax(), "t_ms"]
    return dfl.assign(direction=np.where(dfl["t_ms"] <= peak_t, "up", "down"))


def power_fit(load_g, ohms) -> dict:
    """Least-squares power-law fit R = a * F^b in log-log space (no scipy needed).

    For an FSR, b is negative (resistance falls with force). Returns
    {a, b, r2, n}; r2 is the coefficient of determination of log10(R).
    """
    load_g = np.asarray(load_g, float)
    ohms = np.asarray(ohms, float)
    m = (load_g > 0) & (ohms > 0)
    if m.sum() < 2:
        return dict(a=float("nan"), b=float("nan"), r2=float("nan"), n=int(m.sum()))
    x, y = np.log10(load_g[m]), np.log10(ohms[m])
    b, loga = np.polyfit(x, y, 1)
    yhat = b * x + loga
    ss_res = float(((y - yhat) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return dict(a=float(10 ** loga), b=float(b), r2=float(r2), n=int(m.sum()))


def find_knee(levels: pd.DataFrame, band: float = 1.25, col: str = "median"):
    """Saturation onset = lightest load whose median R is within `band`x the
    floor (min median R). Returns (knee_load, floor_R) or (None, None)."""
    if len(levels) < 2:
        return None, None
    floor = float(levels[col].min())
    load_col = [c for c in levels.columns if c.startswith("load")][0]
    sat = levels[levels[col] <= band * floor]
    knee = float(sat[load_col].min()) if len(sat) else None
    return knee, floor


# --- Run discovery: map physical sensor parts to their capture files ----------

def sensor_label(name: str) -> str:
    """'FSR1' / 'FSR2' / 'FSR3' from a filename token (`fsr`, `fsr2`, ...)."""
    m = re.search(r"_(fsr\d*)_", name)
    tok = m.group(1) if m else "fsr"
    n = tok[3:] or "1"
    return f"FSR{n}"


def discover_runs(kind: str = "sweep", data_dir: Path = DATA_DIR) -> dict[str, list[Path]]:
    """Group capture files by physical sensor for a given `kind` ('sweep' /
    'creep'). Returns {sensor_label: [paths sorted by name]}."""
    out: dict[str, list[Path]] = {}
    for p in sorted(data_dir.glob("*fsr*.csv")):
        if kind not in p.name:
            continue
        out.setdefault(sensor_label(p.name), []).append(p)
    return out
