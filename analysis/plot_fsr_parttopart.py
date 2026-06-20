#!/usr/bin/env python3
"""Part-to-part spread of the FSR402 sweeps — the "relative, not calibrated" figure.

Overlays the R-vs-load up-sweep of every physical FSR part captured (FSR1, FSR2,
FSR3, ...) on one axis and quantifies how the part-to-part spread COLLAPSES from
light contact to seated loads. This is the headline evidence that the chair FSRs
must be read as *relative* pressure/contact, not calibrated force: at a light
touch the parts disagree by ~10-20x, but by ~2 kg they agree to <10%.

Why up-sweep only: mixing loading and unloading directions makes the per-load
median non-monotonic (hysteresis), which muddies a part-to-part comparison. We
take the loading (up) half of each sweep; pass --both to use all loaded samples.

Inputs: every data/*fsr*sweep*.csv, grouped by sensor (fsr -> FSR1, fsr2 -> FSR2,
...). For a sensor with several sweeps the cleanest is preferred (a '*sweep2*'
file over the first messy '*sweep*'); override with --csv SENSOR=path ... .

Writes analysis/figures/fsr_parttopart.png and prints the spread table.

Usage:
    .venv/bin/python analysis/plot_fsr_parttopart.py [--both] [--rfixed 2833] \
        [--csv FSR1=data/...sweep2.csv FSR3=data/...sweep.csv]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import fsr_lib as fl

# distinct, colour-blind-friendly per-sensor colours
SENSOR_COLORS = {"FSR1": "#1f77b4", "FSR2": "#d62728", "FSR3": "#2ca02c",
                 "FSR4": "#9467bd", "FSR5": "#ff7f0e"}


import re

MIN_LOADED = 200  # a file with fewer loaded samples is treated as mid-capture/empty


def _sweep_num(p: Path) -> int:
    m = re.search(r"sweep(\d*)", p.name)
    return int(m.group(1)) if m and m.group(1) else 1


def _loaded_count(p: Path) -> int:
    try:
        return len(fl.loaded(fl.load(p)))
    except Exception:
        return 0


def choose_sweep(paths: list[Path]) -> Path:
    """Prefer the latest repeat ('sweep2' > 'sweep') that actually has data, so a
    just-started (still-empty) capture falls back to the completed sweep. If none
    clears MIN_LOADED, return whichever has the most loaded samples."""
    for p in sorted(paths, key=_sweep_num, reverse=True):
        if _loaded_count(p) >= MIN_LOADED:
            return p
    return max(paths, key=_loaded_count)


# A healthy loading/unloading hysteresis ratio is ~1.2-1.5x; a sensor whose
# up-sweep reads many times its down-sweep at a light load was not seated yet
# (first-contact transient) — the same failure mode as the original VOID FSR3.
SEAT_RATIO = 2.5


def sensor_levels(path: Path, both: bool):
    """Per-ladder-rung resistance for one sensor, with a seating-suspect flag.

    For each rung we keep the up-sweep median (the conventional R-vs-load value)
    but also the down-sweep median; a rung whose up/down ratio exceeds SEAT_RATIO
    is flagged `suspect` — its light-end reading is a first-contact seating
    transient, not true part-to-part spread, so it is excluded from the spread%.
    Returns (levels_df[rung,median,r_down,ratio,suspect,count], floor, npts).
    """
    dfl = fl.split_direction(fl.add_canonical(fl.load(path)))
    rows = []
    for canon, sub in dfl.groupby("load_canon"):
        rung = fl.snap_to_ladder(canon)
        if rung is None:
            continue
        up = sub[sub["direction"] == "up"]["ohms"]
        dn = sub[sub["direction"] == "down"]["ohms"]
        if both or len(up) <= 20:        # use all samples if up-sweep is too thin
            r_main = float(sub["ohms"].median()); n = len(sub)
        else:
            r_main = float(up.median()); n = len(up)
        r_dn = float(dn.median()) if len(dn) else float("nan")
        ratio = r_main / r_dn if r_dn and not np.isnan(r_dn) and r_dn > 0 else float("nan")
        suspect = bool(ratio == ratio and ratio > SEAT_RATIO)  # ratio==ratio: not NaN
        rows.append(dict(rung=rung, median=r_main, r_down=r_dn, ratio=ratio,
                         suspect=suspect, count=n))
    import pandas as pd
    lv = pd.DataFrame(rows)
    if lv.empty:
        return lv, float("nan"), 0
    lv = lv.sort_values("count", ascending=False).drop_duplicates("rung").sort_values("rung")
    floor = float(lv["median"].min())
    return lv.reset_index(drop=True), floor, int(dfl["ohms"].count())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", nargs="*", default=[],
                    help="explicit SENSOR=path overrides, e.g. FSR1=data/...sweep2.csv")
    ap.add_argument("--both", action="store_true",
                    help="use all loaded samples instead of the up-sweep only")
    ap.add_argument("--rfixed", type=float, default=2833.0)
    args = ap.parse_args()

    runs = fl.discover_runs("sweep")
    chosen: dict[str, Path] = {s: choose_sweep(ps) for s, ps in runs.items()}
    for spec in args.csv:  # apply overrides
        sensor, _, path = spec.partition("=")
        chosen[sensor] = Path(path)
    if not chosen:
        print("No data/*fsr*sweep*.csv files found.")
        return 2

    series = {}
    for sensor in sorted(chosen):
        lv, floor, npts = sensor_levels(chosen[sensor], args.both)
        if len(lv):
            series[sensor] = lv
            nsus = int(lv["suspect"].sum())
            warn = f"  ⚠ {nsus} seating-suspect level(s)" if nsus else ""
            print(f"{sensor:5s} {chosen[sensor].name}: {len(lv)} rungs, "
                  f"R {lv['median'].max():.0f} -> {floor:.0f} ohm, {npts} samples{warn}")
            for _, r in lv[lv["suspect"]].iterrows():
                print(f"        ⚠ {int(r['rung'])}g: up={r['median']:.0f} vs down={r['r_down']:.0f} "
                      f"ohm (x{r['ratio']:.1f}) — first-contact seating, not part spread")

    # Spread across sensors at each shared ladder rung (seating-suspect points excluded).
    rungs = sorted(set().union(*[set(lv["rung"]) for lv in series.values()]))
    print("\n load(g) | " + " | ".join(f"{s:>9}" for s in sorted(series)) + " |  spread%  (clean sensors)")
    spread_rows = []
    for rung in rungs:
        clean = {}
        cells = []
        for s in sorted(series):
            lv = series[s]
            row = lv[lv["rung"] == rung]
            if not len(row):
                cells.append(f"{'-':>9}"); continue
            v = float(row["median"].iloc[0]); sus = bool(row["suspect"].iloc[0])
            cells.append(f"{v:8.0f}{'*' if sus else ' '}")
            if not sus:
                clean[s] = v
        cells = " | ".join(cells)
        if len(clean) >= 2:
            spread = 100.0 * (max(clean.values()) - min(clean.values())) / min(clean.values())
            spread_rows.append((rung, spread, len(clean)))
            print(f" {int(rung):7d} | {cells} | {spread:7.0f}%")
        else:
            print(f" {int(rung):7d} | {cells} | {'(<2 clean)':>9}")
    print("  (* = seating-suspect: up/down ratio > %.1f, excluded from spread)" % SEAT_RATIO)

    # --- figure: overlay curves (left) + spread-vs-load (right) ---------------
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5.5))
    for sensor in sorted(series):
        lv = series[sensor]
        c = SENSOR_COLORS.get(sensor, None)
        clean = lv[~lv["suspect"]]; sus = lv[lv["suspect"]]
        a1.plot(lv["rung"], lv["median"], "-", color=c, lw=1.5, alpha=0.6, zorder=1)
        a1.plot(clean["rung"], clean["median"], "o", color=c, ms=6, label=sensor, zorder=3)
        if len(sus):  # hollow markers = seating-suspect up-sweep transient
            a1.plot(sus["rung"], sus["median"], "o", mfc="white", mec=c, mew=1.6, ms=7, zorder=3)
            a1.plot(sus["rung"], sus["r_down"], "v", color=c, ms=6, alpha=0.7, zorder=2)
    a1.plot([], [], "o", mfc="white", mec="grey", label="seating-suspect (up)")
    a1.plot([], [], "v", color="grey", label="↳ its down-sweep")
    a1.set_xscale("log")
    a1.set_yscale("log")
    a1.set_xlabel("applied load [g]")
    a1.set_ylabel("FSR resistance [Ω]  (median per level)")
    a1.set_title("Per-part R-vs-load (up-sweep)" + ("" if not args.both else " — both directions"))
    a1.grid(alpha=0.3, which="both")
    a1.legend(title="physical part", fontsize=9)

    if spread_rows:
        sr = np.array([(r, s) for r, s, _ in spread_rows])
        a2.plot(sr[:, 0], sr[:, 1], "o-", color="k", lw=2, ms=6)
        for r, s, n in spread_rows:
            a2.annotate(f"{s:.0f}%", (r, s), textcoords="offset points",
                        xytext=(0, 8), fontsize=8, ha="center")
        a2.set_xscale("log")
        a2.set_xlabel("applied load [g]")
        a2.set_ylabel("part-to-part spread  (max−min)/min  [%]")
        a2.set_title("Part-to-part spread (well-seated levels only)\nconverges toward the shared floor as load rises")
        a2.grid(alpha=0.3, which="both")
        a2.axhline(10, color="grey", ls=":", lw=1, label="10% band")
        a2.legend(fontsize=9)

    n_sensors = len(series)
    fig.suptitle(f"FSR402 part-to-part spread (N={n_sensors})  (R_fixed = {args.rfixed:.0f} Ω)  — "
                 "all parts converge to a shared ~160–180 Ω floor; seating transients (hollow ▽) "
                 "dwarf true part spread")
    fig.tight_layout()
    fl.FIG_DIR.mkdir(exist_ok=True)
    out = fl.FIG_DIR / "fsr_parttopart.png"
    fig.savefig(out, dpi=130)
    print(f"\n  wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
