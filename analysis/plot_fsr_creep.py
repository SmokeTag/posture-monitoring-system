#!/usr/bin/env python3
"""Plot the FSR402 creep curve: resistance vs. time under a constant dead load.

Input: a capture from firmware/uno_fsr_test in CSV mode — columns
`t_ms,load_kg,raw,ohms` (ohms = -1 means open/no force). Rest a pre-weighed dead
weight on the sensor, stamp its mass once, and leave it untouched ~10 min so the
FSR's slow drift (creep) under sustained load can be measured.

NOTE on units: the precision-balance light-contact protocol stamps the load in
GRAMS even though the column is named `load_kg`. Pass --units g (default) so the
title reports the held mass correctly; the resistance axis is unit-independent.

Robustness: the very end of a creep capture usually contains a few lift-off /
recovery samples (resistance spikes) that are still stamped with the load because
the operator removed the weight without re-typing. We bin the loaded window into
fixed time bins and use the per-bin median, which ignores that tail, and report
creep as (R_last_bin - R_first_bin) / R_first_bin.

Writes analysis/figures/fsr_creep.png and prints the creep summary.

Usage:
    .venv/bin/python analysis/plot_fsr_creep.py [--csv data/<run>.csv] \
        [--rfixed 2833] [--units g] [--load 1000] [--bin-min 0.5]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = REPO_ROOT / "data"
FIG_DIR = Path(__file__).resolve().parent / "figures"
COLS = ["t_ms", "load_kg", "raw", "ohms"]


def default_csv() -> Path:
    files = sorted(DATA_DIR.glob("*creep*.csv")) or sorted(DATA_DIR.glob("*fsr*.csv"))
    return files[-1] if files else DATA_DIR / "<no creep capture yet>.csv"


def load(path) -> pd.DataFrame:
    df = pd.read_csv(path, comment="#", header=None, names=COLS)
    df = df[pd.to_numeric(df["t_ms"], errors="coerce").notna()].copy()
    for c in COLS:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=COLS).reset_index(drop=True)
    df["t_min"] = (df["t_ms"] - df["t_ms"].min()) / 60000.0
    return df


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", default=str(default_csv()))
    ap.add_argument("--rfixed", type=float, default=2833.0, help="divider resistor used (ohms), for the title")
    ap.add_argument("--units", choices=["g", "kg"], default="g", help="units the load column is stamped in")
    ap.add_argument("--load", type=float, default=None, help="held load to isolate (in --units); default = the most common loaded value")
    ap.add_argument("--bin-min", type=float, default=0.5, help="time-bin width in minutes for the robust median")
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print(f"No creep capture found at {path}.\n"
              f"Capture one with firmware/uno_fsr_test (press 'l', rest a dead weight,\n"
              f"type its mass once, leave ~10 min), saving to data/<date>_fsr_*creep*.csv.")
        return 2

    df = load(path)
    loaded = df[(df["ohms"] > 0) & (df["load_kg"] > 0)].copy()
    if loaded.empty:
        print("No loaded samples (ohms>0 & load>0) found — nothing to plot.")
        return 2

    held = args.load if args.load is not None else loaded["load_kg"].mode().iloc[0]
    loaded = loaded[loaded["load_kg"] == held]
    held_kg = held / 1000.0 if args.units == "g" else held

    # Robust per-bin median over the loaded window (ignores lift-off tail spikes).
    loaded["bin"] = (loaded["t_min"] / args.bin_min).round() * args.bin_min
    binned = loaded.groupby("bin")["ohms"].median().reset_index().sort_values("bin")
    r0 = float(binned["ohms"].iloc[0])
    r1 = float(binned["ohms"].iloc[-1])
    span_min = float(binned["bin"].iloc[-1] - binned["bin"].iloc[0])
    creep_pct = 100.0 * (r1 - r0) / r0
    rate = creep_pct / span_min if span_min else float("nan")

    print(f"FSR creep: {path.name}")
    print(f"  held load: {held:g} {args.units} ({held_kg:.3f} kg), {len(loaded)} loaded samples")
    print(f"  window: {span_min:.1f} min, {len(binned)} bins of {args.bin_min:g} min")
    print(f"  R: {r0:.0f} -> {r1:.0f} ohm  (creep {creep_pct:+.1f}% over {span_min:.1f} min, {rate:+.2f} %/min)")
    print(f"  => resistance {'drops' if creep_pct < 0 else 'rises'} under constant load "
          f"=> FSR reads {'more' if creep_pct < 0 else 'less'} force over time")

    # Clip the y-range to the loaded band so the lift-off tail doesn't squash the plot.
    lo, hi = 0.85 * binned["ohms"].min(), 1.15 * binned["ohms"].max()
    fig, ax = plt.subplots(figsize=(11, 5.5))
    ax.scatter(loaded["t_min"], loaded["ohms"], s=8, alpha=0.2, color="#1f77b4", label="samples")
    ax.plot(binned["bin"], binned["ohms"], "o-", color="#d62728", lw=2,
            label=f"median / {args.bin_min:g} min")
    ax.axhline(r0, color="grey", ls=":", lw=1, label=f"start ~{r0:.0f} Ω")
    ax.annotate(f"creep {creep_pct:+.1f}% over {span_min:.1f} min ({rate:+.2f} %/min)",
                xy=(binned["bin"].iloc[-1], r1), xytext=(0.5, 0.92), textcoords="axes fraction",
                fontsize=11, ha="left",
                arrowprops=dict(arrowstyle="->", color="#d62728", alpha=0.6))
    ax.set_ylim(lo, hi)
    ax.set_xlabel("time under load [min]")
    ax.set_ylabel("FSR resistance [Ω]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, loc="lower left")
    fig.suptitle(f"FSR402 creep @ {held_kg:.2f} kg constant dead load  (R_fixed = {args.rfixed:.0f} Ω)  — "
                 "drift under sustained load → reinforces 'relative, not calibrated'")
    fig.tight_layout()
    FIG_DIR.mkdir(exist_ok=True)
    out = FIG_DIR / "fsr_creep.png"
    fig.savefig(out, dpi=130)
    print(f"  wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
