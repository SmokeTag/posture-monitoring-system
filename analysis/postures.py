"""Shared loader + helpers for the IMU posture capture (postures.csv).

The CSV is produced by firmware/xiao_imu_test/xiao_imu_test.ino: a continuous
~20 Hz serial dump from the XIAO nRF52840 Sense worn on the upper back. Every
prose/diagnostic line is prefixed with '#', so the numeric stream parses
directly; the first non-'#' line is the textual header.

Columns: t_ms,tag,cal,pitch,roll,dpitch,droll,ax,ay,az,gx,gy,gz
  t_ms          millis() timestamp since boot
  tag           posture label (digit pressed on serial; holds until changed)
  cal           0 before 'c' calibration, 1 after
  pitch, roll   tilt from gravity, degrees  (forward/back, left/right lean)
  dpitch,droll  deviation from the calibrated "upright" reference (== pitch/roll
                while cal==0, since the reference is 0 until calibrated)
  ax,ay,az      accelerometer, g
  gx,gy,gz      gyroscope, deg/s
"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import pandas as pd

COLS = ["t_ms", "tag", "cal", "pitch", "roll", "dpitch", "droll",
        "ax", "ay", "az", "gx", "gy", "gz"]

# Documented posture tags (firmware header + data/README.md).
DOCUMENTED_TAGS = {0, 1, 2, 3, 4, 5, 9}
JUNK_TAG = 9  # "moving / transition" — dropped from steady-posture analysis

# Pitch convention for this mounting: upright ~45 deg; leaning FORWARD lowers
# pitch (slouch ~20, head-down ~27), leaning BACK raises it (laid-back ~64).
TAG_LABELS = {
    0: "Upright",
    1: "Forward slouch",
    2: "Lean left",
    3: "Lean right",
    4: "Head-down",
    5: "Laid-back",   # hips forward, torso back — highest pitch
    9: "Transition / moving",
}

TAG_COLORS = {
    0: "#2ca02c",  # green  — the "good" reference posture
    1: "#ff7f0e",  # orange
    2: "#1f77b4",  # blue
    3: "#9467bd",  # purple
    4: "#d62728",  # red
    5: "#8c564b",  # brown  — undocumented
    9: "#cfcfcf",  # grey   — junk/motion
}

# Repo root = parent of the analysis/ directory this file lives in.
REPO_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = REPO_ROOT / "data"

RAD_TO_DEG = 57.2957795


def latest_csv() -> Path:
    """Newest capture under data/ (ISO-dated names sort chronologically).

    Falls back to a legacy ./postures.csv so older invocations still resolve.
    The returned path may not exist; load() will then raise a clear error.
    """
    files = sorted(DATA_DIR.glob("*.csv"))
    if files:
        return files[-1]
    return REPO_ROOT / "postures.csv"


DEFAULT_CSV = latest_csv()


def load(path: str | Path = DEFAULT_CSV) -> pd.DataFrame:
    """Parse postures.csv into a clean, fully-numeric, file-ordered frame."""
    df = pd.read_csv(path, comment="#", header=None, names=COLS)
    # Drop the textual header row (its t_ms cell is the string "t_ms").
    df = df[pd.to_numeric(df["t_ms"], errors="coerce").notna()].copy()
    for c in COLS:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df["tag"] = df["tag"].astype(int)
    df["cal"] = df["cal"].astype(int)
    return df.reset_index(drop=True)


def steady(df: pd.DataFrame) -> pd.DataFrame:
    """Rows belonging to a held posture (transitions/motion dropped)."""
    return df[df["tag"] != JUNK_TAG].copy()


def accel_magnitude(df: pd.DataFrame) -> pd.Series:
    """|g| = sqrt(ax^2+ay^2+az^2). ~1.0 at rest; departs from 1 under motion."""
    return np.sqrt(df["ax"] ** 2 + df["ay"] ** 2 + df["az"] ** 2)


def deviation(df: pd.DataFrame) -> pd.Series:
    """Posture-alert metric: sqrt(dpitch^2 + droll^2), the angular distance from
    the calibrated upright. ~0 when upright, large for a held bad posture.
    Only meaningful on cal==1 rows (otherwise dpitch/droll == absolute angle)."""
    return np.sqrt(df["dpitch"] ** 2 + df["droll"] ** 2)


def recompute_angles(df: pd.DataFrame) -> tuple[pd.Series, pd.Series]:
    """Re-derive pitch/roll from accel with the firmware's exact formulas."""
    pitch = np.arctan2(-df["ax"], np.sqrt(df["ay"] ** 2 + df["az"] ** 2)) * RAD_TO_DEG
    roll = np.arctan2(df["ay"], df["az"]) * RAD_TO_DEG
    return pitch, roll


def segments(df: pd.DataFrame) -> list[dict]:
    """Split into contiguous runs of constant tag (one held posture each)."""
    tag = df["tag"].to_numpy()
    t = df["t_ms"].to_numpy()
    cal = df["cal"].to_numpy()
    change = np.flatnonzero(np.diff(tag) != 0) + 1
    starts = np.concatenate(([0], change))
    ends = np.concatenate((change, [len(df)]))  # end-exclusive
    out = []
    for s, e in zip(starts, ends):
        out.append(dict(
            tag=int(tag[s]), i0=int(s), i1=int(e - 1), n=int(e - s),
            t0=float(t[s]), t1=float(t[e - 1]),
            dur_s=float((t[e - 1] - t[s]) / 1000.0), cal=int(cal[s]),
        ))
    return out


def calibration_events(df: pd.DataFrame, gap_ms: float = 200.0) -> pd.DataFrame:
    """Calibration presses, detected as the timing gaps they create.

    captureReference() blocks ~200 ms (20 samples x delay(10)), so every 'c'
    press shows up as a dt > ~200 ms gap. Returns the rows just after each gap.
    """
    dt = df["t_ms"].diff()
    idx = df.index[dt > gap_ms]
    return df.loc[idx, ["t_ms", "tag", "cal"]].assign(dt_ms=dt.loc[idx])


def calibration_epochs(df: pd.DataFrame, tol: float = 0.05) -> list[dict]:
    """Recover the per-re-don reference plateaus from the data itself.

    Each 'c' press re-zeros (pitchRef, rollRef); since dpitch=pitch-pitchRef,
    the back-derived reference (pitch - dpitch) is piecewise-constant and jumps
    at every re-don. The `cal` column can't do this (it flips 0->1 once and
    never resets), so this is the only in-data way to delimit the re-don epochs.
    Returns one dict per plateau among cal==1 rows: {i0,i1,n,pitch_ref,roll_ref,
    ref_spread} where ref_spread is the within-plateau max-min of pitch_ref
    (should be ~0 if the firmware subtracts a true constant).
    """
    c1 = df[df["cal"] == 1]
    if c1.empty:
        return []
    refp = (c1["pitch"] - c1["dpitch"]).to_numpy()
    refr = (c1["roll"] - c1["droll"]).to_numpy()
    idx = c1.index.to_numpy()
    jump = (np.abs(np.diff(refp)) > tol) | (np.abs(np.diff(refr)) > tol)
    bnd = np.flatnonzero(jump) + 1
    starts = np.concatenate(([0], bnd))
    ends = np.concatenate((bnd, [len(c1)]))
    out = []
    for s, e in zip(starts, ends):
        seg_p = refp[s:e]
        out.append(dict(
            i0=int(idx[s]), i1=int(idx[e - 1]), n=int(e - s),
            pitch_ref=float(seg_p[0]), roll_ref=float(refr[s]),
            ref_spread=float(seg_p.max() - seg_p.min()),
        ))
    return out


def redon_upright_refs(df: pd.DataFrame) -> pd.DataFrame:
    """Genuine re-don 'upright' references for the OE1 mounting-drift result.

    Each 'c' press opens a calibration epoch (calibration_epochs); a *genuine*
    re-don is one where an upright posture (tag==0) was actually held after the
    press. The immediate double-presses capture only tag==9 motion and are
    dropped. The reference is the mean absolute pitch/roll over that epoch's
    tag==0 hold -- the exact quantity the partial report's OE1 drift
    (8.0 deg pitch / 21.6 deg roll, n=4) is computed from (re-don figure). This
    differs from the firmware-snapshot reference (pitch-dpitch), which over all
    7 presses gives the looser 7.7/21.4. Returns one row per genuine epoch,
    indexed 0..N-1 as 'event'.
    """
    rows = []
    for ep in calibration_epochs(df):
        hold = df.loc[ep["i0"]:ep["i1"]]
        hold = hold[hold["tag"] == 0]
        if len(hold):
            rows.append((float(hold["pitch"].mean()),
                         float(hold["roll"].mean()), int(len(hold))))
    return (pd.DataFrame(rows, columns=["pitch_ref", "roll_ref", "n"])
            .reset_index(names="event"))


def parse_calibration_comments(path: str | Path = DEFAULT_CSV) -> pd.DataFrame:
    """Extract '# calibrated upright: pitch=.. roll=..' lines, in file order.

    These are the per-re-don upright references for the mounting-drift (OE1)
    result. They carry no timestamp, so they are returned indexed 1..N.
    """
    pat = re.compile(r"calibrated upright:\s*pitch=([-\d.]+)\s*deg\s+roll=([-\d.]+)")
    rows = []
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                continue
            m = pat.search(line)
            if m:
                rows.append((float(m.group(1)), float(m.group(2))))
    return pd.DataFrame(rows, columns=["pitch_ref", "roll_ref"]).reset_index(names="event")
