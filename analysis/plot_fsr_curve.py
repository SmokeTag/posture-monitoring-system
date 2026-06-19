#!/usr/bin/env python3
"""Plot the FSR402 resistance-vs-load curve and find the saturation knee.

Input: a capture from firmware/uno_fsr_test in CSV mode — columns
`t_ms,load_kg,raw,ohms` (ohms = -1 means open/no force). Press the FSR through a
rigid puck on a bathroom scale, type each scale reading to stamp `load_kg`.

Writes analysis/figures/fsr_curve.png and prints the saturation summary.

Usage:
    .venv/bin/python analysis/plot_fsr_curve.py [--csv data/<run>.csv] [--rfixed 3300]
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
G = 9.80665  # kg -> N
COLS = ["t_ms", "load_kg", "raw", "ohms"]


def default_csv() -> Path:
    files = sorted(DATA_DIR.glob("*fsr*.csv"))
    return files[-1] if files else DATA_DIR / "<no fsr capture yet>.csv"


def load(path) -> pd.DataFrame:
    df = pd.read_csv(path, comment="#", header=None, names=COLS)
    df = df[pd.to_numeric(df["t_ms"], errors="coerce").notna()].copy()
    for c in COLS:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    return df.reset_index(drop=True)


def per_level(df: pd.DataFrame) -> pd.DataFrame:
    """Median resistance per stamped load level (loaded samples only)."""
    loaded = df[(df["ohms"] > 0) & (df["load_kg"] > 0)]
    g = loaded.groupby("load_kg")["ohms"].agg(["median", "std", "count"])
    return g.reset_index().sort_values("load_kg")


def find_knee(levels: pd.DataFrame, band: float = 1.25):
    """Saturation onset = lightest load whose median R is within `band`x the
    minimum median R (i.e. the curve has flattened onto its floor)."""
    if len(levels) < 2:
        return None, None
    floor = levels["median"].min()
    sat = levels[levels["median"] <= band * floor]
    knee = float(sat["load_kg"].min()) if len(sat) else None
    return knee, float(floor)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", default=str(default_csv()))
    ap.add_argument("--rfixed", type=float, default=3300.0, help="divider resistor used (ohms), for the title")
    ap.add_argument("--units", choices=["kg", "g"], default="kg",
                    help="units the load column is stamped in (the precision-balance protocol uses grams)")
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print(f"No FSR capture found at {path}.\n"
              f"Capture one with firmware/uno_fsr_test (press 'l', type each scale\n"
              f"reading in kg), saving to data/<date>_fsr_*.csv, then pass --csv.")
        return 2

    df = load(path)
    if args.units == "g":  # protocol stamps grams into the load_kg column; normalize to kg
        df["load_kg"] = df["load_kg"] / 1000.0
    levels = per_level(df)
    n_open = int((df["ohms"] <= 0).sum())
    knee, floor = find_knee(levels)

    print(f"FSR curve: {path.name}")
    print(f"  {len(df)} samples, {len(levels)} load levels, {n_open} open/no-force samples")
    if len(levels):
        print(f"  R range: {levels['median'].max():.0f} -> {levels['median'].min():.0f} ohm "
              f"(load {levels['load_kg'].min():.1f} -> {levels['load_kg'].max():.1f} kg)")
    if knee is not None:
        print(f"  saturation knee ~ {knee:.1f} kg; floor R ~ {floor:.0f} ohm")

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5.5))
    loaded = df[(df["ohms"] > 0) & (df["load_kg"] > 0)]
    for ax, logy in ((a1, False), (a2, True)):
        ax.scatter(loaded["load_kg"], loaded["ohms"], s=10, alpha=0.25,
                   color="#1f77b4", label="samples")
        if len(levels):
            ax.plot(levels["load_kg"], levels["median"], "o-", color="#d62728",
                    lw=2, label="median per load")
            ax.errorbar(levels["load_kg"], levels["median"], yerr=levels["std"].fillna(0),
                        fmt="none", ecolor="#d62728", alpha=0.5, capsize=3)
        if knee is not None:
            ax.axvline(knee, color="k", ls="--", lw=1.2, label=f"saturation ~{knee:.0f} kg")
            ax.axhline(floor, color="grey", ls=":", lw=1, label=f"floor ~{floor:.0f} Ω")
        if logy:
            ax.set_yscale("log")
            ax.set_title("log scale (saturation = flattening)")
        else:
            ax.set_title("linear scale")
        ax.set_xlabel("applied load [kg]")
        ax.set_ylabel("FSR resistance [Ω]")
        ax.grid(alpha=0.3, which="both")
        ax.legend(fontsize=8)
        # secondary force axis in Newtons
        sec = ax.secondary_xaxis("top", functions=(lambda k: k * G, lambda n: n / G))
        sec.set_xlabel("applied force [N]")
    fig.suptitle(f"FSR402 resistance vs. load  (R_fixed = {args.rfixed:.0f} Ω)  — "
                 "saturates → treat as relative pressure/contact, not calibrated force")
    fig.tight_layout()
    FIG_DIR.mkdir(exist_ok=True)
    out = FIG_DIR / "fsr_curve.png"
    fig.savefig(out, dpi=130)
    print(f"  wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
