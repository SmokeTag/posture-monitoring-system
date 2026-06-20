#!/usr/bin/env python3
"""Plot the FSR402 loading vs unloading hysteresis loop for one sweep.

A sweep goes light -> heavy -> light. The same FSR reads a DIFFERENT resistance
at the same load depending on whether you are loading (up) or unloading (down) —
the up/down loop. This figure splits the sweep by direction (via time order, the
peak-load instant) and overlays the two branches, quantifying the gap per load.

The lesson for the project: the loop is wide at light contact (tens of %) and
collapses to a few % by ~1 kg, so all the "which way were you pressing?" ambiguity
lives at the light end — the seated/operating regime is direction-independent.
A *pathologically* wide loop at the light end (up >> down) is instead a seating
artifact (the puck not yet bedded into the pad); validate_fsr.py flags that.

Writes analysis/figures/fsr_hysteresis.png and prints the per-load loop table.

Usage:
    .venv/bin/python analysis/plot_fsr_hysteresis.py [--csv data/<sweep>.csv] [--rfixed 2833]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import fsr_lib as fl


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # FSR2's sweep shows the widest, cleanest loop; fall back to the newest sweep.
    default = fl.DATA_DIR / "2026-06-19_fsr2_andre_2833_sweep.csv"
    if not default.exists():
        default = fl.latest_csv("*fsr*sweep*.csv")
    ap.add_argument("--csv", default=str(default))
    ap.add_argument("--rfixed", type=float, default=2833.0)
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print(f"No sweep capture at {path}.")
        return 2

    dfl = fl.split_direction(fl.add_canonical(fl.load(path)))
    rows = []
    for canon, sub in dfl.groupby("load_canon"):
        up = sub[sub["direction"] == "up"]["ohms"]
        dn = sub[sub["direction"] == "down"]["ohms"]
        if len(up) < 3 or len(dn) < 3:
            continue
        ru, rd = float(up.median()), float(dn.median())
        rows.append((int(canon), ru, rd, 100.0 * (rd - ru) / ru, len(up), len(dn)))
    rows.sort()
    if not rows:
        print("Not enough up AND down samples per level to show a loop "
              "(is this a full up-then-down sweep?).")
        return 2

    print(f"FSR hysteresis: {path.name}")
    print("  load(g) |  R_up |  R_dn | (dn-up)/up")
    for g, ru, rd, h, nu, nd in rows:
        print(f"  {g:7d} | {ru:5.0f} | {rd:5.0f} | {h:+7.1f}%")
    light = [h for g, ru, rd, h, nu, nd in rows if g <= 400]
    heavy = [h for g, ru, rd, h, nu, nd in rows if g >= 1000]
    if light:
        print(f"  mean |hysteresis| <=400 g: {np.mean(np.abs(light)):.0f}%")
    if heavy:
        print(f"  mean |hysteresis| >=1000 g: {np.mean(np.abs(heavy)):.0f}%  (collapses at load)")

    a = np.array([(g, ru, rd, h) for g, ru, rd, h, nu, nd in rows], float)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5.5))
    a1.plot(a[:, 0], a[:, 1], "o-", color="#1f77b4", lw=2, label="loading (up)")
    a1.plot(a[:, 0], a[:, 2], "s--", color="#d62728", lw=2, label="unloading (down)")
    a1.fill_between(a[:, 0], a[:, 1], a[:, 2], color="grey", alpha=0.18, label="loop")
    a1.set_xscale("log"); a1.set_yscale("log")
    a1.set_xlabel("applied load [g]"); a1.set_ylabel("FSR resistance [Ω]")
    a1.set_title("Up vs down branch — the hysteresis loop")
    a1.grid(alpha=0.3, which="both"); a1.legend(fontsize=9)

    a2.plot(a[:, 0], a[:, 3], "o-", color="k", lw=2)
    a2.axhline(0, color="grey", lw=1)
    a2.axhspan(-5, 5, color="green", alpha=0.10, label="±5% (negligible)")
    a2.set_xscale("log")
    a2.set_xlabel("applied load [g]")
    a2.set_ylabel("hysteresis  (R_down − R_up)/R_up  [%]")
    a2.set_title("Loop width collapses with load\n→ direction-independent in the operating regime")
    a2.grid(alpha=0.3, which="both"); a2.legend(fontsize=9)

    fig.suptitle(f"FSR402 loading/unloading hysteresis — {path.name}  (R_fixed = {args.rfixed:.0f} Ω)")
    fig.tight_layout()
    fl.FIG_DIR.mkdir(exist_ok=True)
    out = fl.FIG_DIR / "fsr_hysteresis.png"
    fig.savefig(out, dpi=130)
    print(f"  wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
