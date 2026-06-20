#!/usr/bin/env python3
"""Fit a response model to the FSR402 sweep and show why it must be read as
relative pressure, not calibrated force.

Two panels:
  (left)  R vs load, log-log, with a power-law fit R = a·F^b (numpy, no scipy)
          and its R². The 2 kg scale cap is marked: the measured "floor" is the
          resistance at the heaviest load we could reach, an UPPER BOUND on the
          true saturation floor — a seated adult loads well past this, so the
          real operating point sits in the shaded, unmeasured >2 kg region.
  (right) Conductance G = 1/R vs load, linear axes, with a linear fit. FSR
          datasheets model conductance as ~linear in force; this panel shows G-vs-F
          is far more linear (higher R²) than R-vs-F, i.e. conductance is the
          natural variable for a relative-pressure map.

Also prints: the >1 kg operating-band sensitivity (Ω per gram) and the ADC
resolution (Ω per count) at light load vs at the floor, to quantify how few
distinct pressure bands the divider can actually resolve when seated.

Writes analysis/figures/fsr_model.png.

Usage:
    .venv/bin/python analysis/plot_fsr_model.py [--csv data/<sweep>.csv] [--rfixed 2833]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import fsr_lib as fl


def linfit(x, y):
    """Linear y = m·x + c with R²."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    m, c = np.polyfit(x, y, 1)
    yhat = m * x + c
    ss_res = float(((y - yhat) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    return m, c, (1 - ss_res / ss_tot if ss_tot else float("nan"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    default = fl.DATA_DIR / "2026-06-19_fsr_andre_2833_sweep2.csv"
    if not default.exists():
        default = fl.latest_csv("*fsr*sweep*.csv")
    ap.add_argument("--csv", default=str(default))
    ap.add_argument("--rfixed", type=float, default=2833.0)
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print(f"No sweep capture at {path}.")
        return 2

    # Use the up-sweep per-level medians (avoids hysteresis muddying the fit).
    dfl = fl.split_direction(fl.add_canonical(fl.load(path)))
    up = dfl[dfl["direction"] == "up"]
    lv = fl.per_level(up if len(up) > 20 else dfl).sort_values("load_canon")
    load = lv["load_canon"].to_numpy(float)
    R = lv["median"].to_numpy(float)
    G = 1.0 / R

    pw = fl.power_fit(load, R)
    gm, gc, gr2 = linfit(load, G)               # conductance vs force (linear)
    rm, rc, rr2 = linfit(load, R)               # resistance vs force (linear) for contrast

    maxload = float(load.max())
    floor = float(R.min())

    # Operating-band sensitivity (>= 1 kg) and ADC resolution.
    op = lv[lv["load_canon"] >= 1000]
    if len(op) >= 2:
        sens = float(np.polyfit(op["load_canon"], op["median"], 1)[0])  # Ω/g
    else:
        sens = float("nan")
    # |dR/dADC| = R_FIXED * ADC_MAX / raw^2 from the divider law.
    def res_at(raw):
        return args.rfixed * fl.ADC_MAX / (raw ** 2)
    raw_light = float(lv["raw_med"].iloc[0]); raw_floor = float(lv["raw_med"].iloc[-1])

    print(f"FSR model: {path.name}")
    print(f"  power law   R = {pw['a']:.0f}·F^({pw['b']:.3f})   R²={pw['r2']:.3f}  (n={pw['n']})")
    print(f"  conductance G = {gm:.3e}·F + {gc:.3e}   R²={gr2:.3f}   "
          f"(vs R-linear R²={rr2:.3f} → G is the linear variable)")
    print(f"  operating band (>=1 kg) sensitivity: {sens:.3f} Ω/g "
          f"({abs(sens)*100:.1f} Ω per 100 g)")
    print(f"  ADC resolution: ~{res_at(raw_light):.1f} Ω/count at light load (raw {raw_light:.0f}), "
          f"~{res_at(raw_floor):.1f} Ω/count at floor (raw {raw_floor:.0f})")
    print(f"  → at the {floor:.0f} Ω floor that is ~{100*res_at(raw_floor)/floor:.1f}% per count: "
          f"only a handful of resolvable pressure bands when seated.")

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13.5, 5.5))
    # left: R vs load log-log + power fit + cap
    a1.plot(load, R, "o", color="#1f77b4", ms=7, label="median per level (up-sweep)")
    xs = np.logspace(np.log10(load.min()), np.log10(load.max()), 100)
    a1.plot(xs, pw["a"] * xs ** pw["b"], "-", color="#d62728", lw=2,
            label=f"R = {pw['a']:.0f}·F^{pw['b']:.2f}  (R²={pw['r2']:.2f})")
    a1.axvline(maxload, color="k", ls="--", lw=1.2)
    a1.axvspan(maxload, maxload * 8, color="orange", alpha=0.12)
    a1.text(maxload * 1.1, R.max() * 0.6, "seated load\n(>2 kg, unmeasured)",
            fontsize=8, color="#a05a00")
    a1.set_xscale("log"); a1.set_yscale("log")
    a1.set_xlabel("applied load [g]"); a1.set_ylabel("FSR resistance [Ω]")
    a1.set_title("Power-law fit + scale cap")
    a1.grid(alpha=0.3, which="both"); a1.legend(fontsize=8)

    # right: conductance vs load linear + linear fit
    a2.plot(load, G * 1e3, "o", color="#2ca02c", ms=7, label="G = 1/R")
    a2.plot(load, (gm * load + gc) * 1e3, "-", color="#d62728", lw=2,
            label=f"linear fit R²={gr2:.2f}")
    a2.set_xlabel("applied load [g]"); a2.set_ylabel("conductance G = 1/R [mS]")
    a2.set_title(f"Conductance is ~linear in force (R²={gr2:.2f})\n"
                 f"vs resistance-linear R²={rr2:.2f}")
    a2.grid(alpha=0.3); a2.legend(fontsize=9)

    fig.suptitle(f"FSR402 response model — {path.name}  (R_fixed = {args.rfixed:.0f} Ω)  — "
                 "fit holds, but the floor is a cap reading → relative pressure only")
    fig.tight_layout()
    fl.FIG_DIR.mkdir(exist_ok=True)
    out = fl.FIG_DIR / "fsr_model.png"
    fig.savefig(out, dpi=130)
    print(f"  wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
