#!/usr/bin/env python3
"""Validate an FSR402 capture (sweep or creep) and print a PASS/WARN/FAIL report.

Turn-key gate for Campaign B captures from firmware/uno_fsr_test: run it right
after a capture to know whether the run is good BEFORE analysing or committing
it. It auto-detects the capture kind and tunes the checks:

  SWEEP  — enough load levels, real dynamic range, R drops monotonically with
           load, raw rails toward the floor, floor in the expected band, and a
           SEATING check (up/down hysteresis ratio) that catches a poorly-seated
           puck — the exact failure that VOIDed the first FSR3 sweep (raw stuck
           at ~494, R frozen at ~3 kΩ, no real loading).
  CREEP  — one dominant held load, sensible duration/rate, drift magnitude, and
           a lift-off tail flag.

Exit code: 0 if no FAIL, 1 if any check FAILs (WARN does not fail the run).

Usage:
    .venv/bin/python analysis/validate_fsr.py --csv data/<run>.csv
    .venv/bin/python analysis/validate_fsr.py            # newest data/*fsr*.csv
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

import fsr_lib as fl

_counts = {"PASS": 0, "WARN": 0, "FAIL": 0}

# Expectations grounded in the FSR1/FSR2 reference sweeps (see data/README.md).
FLOOR_BAND = (120.0, 300.0)   # Ω the curve should flatten onto at ~2 kg
RAW_RAILED = 900              # heavy load should drive the ADC node this high
MIN_DYNRANGE = 5.0            # Rmax/Rmin over the sweep (VOID capture ~1)
MIN_LEVELS = 4                # distinct load levels with enough samples
MIN_PER_LEVEL = 15            # samples held per level
REACH_G = 1500               # a sweep should reach at least this load
SEAT_RATIO = 2.5             # up/down R ratio above this = first-contact seating


def report(status: str, name: str, detail: str = "") -> None:
    _counts[status] += 1
    line = f"  [{status}] {name}"
    print(line if not detail else f"{line}\n          {detail}")


def section(title: str) -> None:
    print(f"\n{title}\n" + "-" * len(title))


def detect_kind(df, dfl) -> str:
    """'creep' if one load dominates a long held window; else 'sweep'."""
    if dfl.empty:
        return "empty"
    canon = fl.add_canonical(df)
    counts = canon["load_canon"].value_counts()
    n_levels = int((counts >= MIN_PER_LEVEL).sum())
    top_frac = float(counts.iloc[0] / counts.sum())
    span_min = float(dfl["t_min"].max() - dfl["t_min"].min())
    if n_levels <= 2 and top_frac > 0.6 and span_min > 2.0:
        return "creep"
    return "sweep"


def common_checks(df, dfl, args):
    section("1. Structure & parse")
    n = len(df)
    report("PASS" if n > 0 else "FAIL", f"parsed {n} numeric rows")
    nl = len(dfl)
    report("PASS" if nl > 50 else ("WARN" if nl else "FAIL"),
           f"{nl} loaded samples (ohms>0 & load>0)",
           "few loaded samples — was the load stamped and the sensor pressed?" if nl < 50 else "")
    # field-count sanity on raw numeric lines (4 cols)
    bad = 0
    with open(args.csv) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.split(",")[0] == "t_ms":
                continue
            if "\t" in line:           # leftover human-readable bring-up line
                continue
            if line.count(",") != len(fl.COLS) - 1:
                bad += 1
    report("PASS" if bad == 0 else "WARN", "data lines have 4 comma-separated fields",
           f"{bad} odd lines (serial interleaving; the loader drops them)" if bad else "")

    section("2. Timing & sampling")
    dt = df["t_ms"].diff()
    neg_idx = list(df.index[dt < 0])
    first_loaded = dfl.index.min() if len(dfl) else len(df)
    # A negative step before any load is stamped is just the capture-start/boot
    # boundary (stale millis from the previous session, then the board reboots);
    # only a reset INSIDE the loaded window corrupts the data.
    mid_resets = [i for i in neg_idx if i > first_loaded]
    boot = len(neg_idx) - len(mid_resets)
    if mid_resets:
        report("FAIL", "t_ms non-decreasing within the loaded window",
               f"{len(mid_resets)} reset(s) mid-capture — timing/creep math is corrupted")
    else:
        report("PASS", "t_ms non-decreasing within the loaded window",
               f"{boot} pre-load boot boundary ignored (stale millis before 'l')" if boot else "")
    med = dt.median() if len(dt) else float("nan")
    report("PASS", f"median dt = {med:.0f} ms (~{1000/med:.1f} Hz)" if med else "no timing",
           f"loaded window {dfl['t_min'].max()-dfl['t_min'].min():.1f} min" if nl else "")


def sweep_checks(df, dfl, args):
    section("3. Load coverage")
    canon = fl.add_canonical(df)
    lv = fl.per_level(canon)
    lv = lv[lv["count"] >= MIN_PER_LEVEL]
    report("PASS" if len(lv) >= MIN_LEVELS else "WARN",
           f"{len(lv)} load levels with >= {MIN_PER_LEVEL} samples",
           f"levels (g): {[int(x) for x in lv['load_canon']]}")
    maxload = float(canon["load_canon"].max()) if len(canon) else 0
    report("PASS" if maxload >= REACH_G else "WARN",
           f"sweep reaches {maxload:.0f} g",
           f"want >= {REACH_G} g to approach saturation" if maxload < REACH_G else "")

    section("4. Loading sanity  (catches a VOID / not-pressed capture)")
    rawmax = float(df["raw"].max())
    report("PASS" if rawmax >= RAW_RAILED else "FAIL",
           f"raw ADC reaches {rawmax:.0f} / {fl.ADC_MAX} under load",
           "raw never gets high -> the puck was not transferring force (VOID-style)"
           if rawmax < RAW_RAILED else "")
    rmin, rmax = float(lv["median"].min()), float(lv["median"].max())
    dyn = rmax / rmin if rmin else float("inf")
    report("PASS" if dyn >= MIN_DYNRANGE else "FAIL",
           f"resistance dynamic range x{dyn:.0f}  ({rmax:.0f} -> {rmin:.0f} Ω)",
           "R barely changes with load -> not actually loaded (VOID-style)"
           if dyn < MIN_DYNRANGE else "")
    in_band = FLOOR_BAND[0] <= rmin <= FLOOR_BAND[1]
    report("PASS" if in_band else "WARN",
           f"floor R = {rmin:.0f} Ω in expected {FLOOR_BAND[0]:.0f}-{FLOOR_BAND[1]:.0f} Ω band",
           "floor outside band -> wrong R_FIXED, bad contact, or unsaturated" if not in_band else "")

    section("5. Curve shape & saturation")
    up = fl.split_direction(canon)
    up = up[up["direction"] == "up"]
    ulv = fl.per_level(up).sort_values("load_canon") if len(up) else lv
    drops = np.diff(ulv["median"].to_numpy())
    inversions = int((drops > 0.05 * ulv["median"].to_numpy()[:-1]).sum())
    report("PASS" if inversions <= 1 else "WARN",
           f"up-sweep R decreases monotonically with load ({inversions} inversion(s))",
           "inversions usually mean hysteresis/seating noise at the light end" if inversions > 1 else "")
    knee, floor = fl.find_knee(lv)
    if knee is not None:
        capped = abs(knee - maxload) < 1e-6 or floor == rmin and maxload <= REACH_G + 600
        report("PASS", f"R within 1.25x of floor by ~{knee:.0f} g (floor {floor:.0f} Ω)",
               "NOTE: floor is the resistance at the heaviest load reached, an UPPER BOUND on the "
               "true saturation floor — a seated adult loads well beyond this scale, so saturation "
               "is even deeper. Report it as 'relative pressure', not a calibrated floor." )

    section("6. Seating health  (up/down hysteresis ratio per level)")
    bad_seat = []
    for canon_load, sub in canon.groupby("load_canon"):
        d = fl.split_direction(canon[canon["load_canon"] == canon_load])
        u = d[d["direction"] == "up"]["ohms"]; dn = d[d["direction"] == "down"]["ohms"]
        if len(u) < 5 or len(dn) < 5:
            continue
        ratio = float(u.median() / dn.median()) if dn.median() > 0 else float("nan")
        if ratio == ratio and ratio > SEAT_RATIO:
            bad_seat.append((int(canon_load), u.median(), dn.median(), ratio))
    if not bad_seat:
        report("PASS", "no level shows a seating transient (all up/down ratios < %.1f)" % SEAT_RATIO)
    else:
        light = [b for b in bad_seat if b[0] <= 600]
        status = "FAIL" if len(bad_seat) >= 3 else "WARN"
        report(status, f"{len(bad_seat)} level(s) read a first-contact seating transient",
               "; ".join(f"{g}g up={u:.0f}/down={dn:.0f}Ω (x{r:.1f})" for g, u, dn, r in bad_seat)
               + " — the puck had not bedded into the pad on the up-sweep; re-seat & recapture if this"
                 " is the light end you care about, or trust the down-sweep there.")


def creep_checks(df, dfl, args):
    section("3. Held load & drift (creep)")
    held = float(dfl["load_kg"].mode().iloc[0])
    sub = dfl[dfl["load_kg"] == held].copy()
    report("PASS" if len(sub) > 100 else "WARN",
           f"dominant held load {held:.0f} g, {len(sub)} samples")
    span = float(sub["t_min"].max() - sub["t_min"].min())
    report("PASS" if span >= 5 else "WARN", f"held window {span:.1f} min",
           "short window (<5 min) — early drift only; don't compare totals to a 10-min run" if span < 5 else "")
    sub["bin"] = (sub["t_min"] / 0.5).round() * 0.5
    binned = sub.groupby("bin")["ohms"].median().sort_index()
    if len(binned) >= 2:
        r0, r1 = float(binned.iloc[0]), float(binned.iloc[-1])
        creep = 100 * (r1 - r0) / r0
        report("PASS" if abs(creep) < 25 else "WARN",
               f"creep {creep:+.1f}% over {span:.1f} min ({creep/span:+.2f} %/min), R {r0:.0f}->{r1:.0f} Ω",
               "drift under constant load -> reinforces 'relative, not calibrated'")
    # lift-off tail: end-of-file spikes still stamped with the load
    tail = sub.tail(max(5, len(sub)//50))
    spike = float(tail["ohms"].median() / binned.median()) if len(binned) else 1.0
    report("WARN" if spike > 1.5 else "PASS",
           "lift-off tail check",
           f"end samples ~x{spike:.1f} the held median — the weight was lifted while still stamped; "
           "per-bin median (plot_fsr_creep.py) ignores it" if spike > 1.5 else "no lift-off spike")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", default=str(fl.latest_csv()), help="path to the FSR capture CSV")
    args = ap.parse_args()

    print(f"Validating: {args.csv}")
    df = fl.load(args.csv)
    dfl = fl.loaded(df)
    kind = detect_kind(df, dfl)
    print(f"Detected capture kind: {kind.upper()}")

    common_checks(df, dfl, args)
    if kind == "creep":
        creep_checks(df, dfl, args)
    elif kind == "sweep":
        sweep_checks(df, dfl, args)
    else:
        report("FAIL", "no usable loaded samples — empty or never-pressed capture")

    section("Verdict")
    print(f"  PASS {_counts['PASS']}   WARN {_counts['WARN']}   FAIL {_counts['FAIL']}")
    if _counts["FAIL"]:
        print("  => FAIL: capture is botched or not truly loaded (see above). Recapture.")
        return 1
    if _counts["WARN"]:
        print("  => USABLE with caveats (light-end seating/hysteresis is expected on FSRs).")
    else:
        print("  => CLEAN.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
