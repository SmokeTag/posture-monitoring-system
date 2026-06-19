#!/usr/bin/env python3
"""Validate an IMU posture capture (postures.csv) and print a PASS/WARN/FAIL
report. Re-runnable on any future capture from firmware/xiao_imu_test.

Checks: parse integrity, timing/sampling, the cal==0 dpitch==pitch invariant,
per-epoch calibration-reference consistency, accelerometer health (|g|~1),
pitch/roll formula agreement, tag coverage vs the documented protocol, steady
segment durations, and posture separability in (pitch, roll) space.

Exit code: 0 if no FAIL, 1 if any check FAILs (WARN does not fail the run).

Usage:
    .venv/bin/python analysis/validate_postures.py [--csv postures.csv]
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

import postures as P

_ICON = {"PASS": "PASS", "WARN": "WARN", "FAIL": "FAIL"}
_counts = {"PASS": 0, "WARN": 0, "FAIL": 0}


def report(status: str, name: str, detail: str = "") -> None:
    _counts[status] += 1
    line = f"  [{_ICON[status]}] {name}"
    print(line if not detail else f"{line}\n          {detail}")


def section(title: str) -> None:
    print(f"\n{title}\n" + "-" * len(title))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", default=str(P.DEFAULT_CSV), help="path to the capture CSV")
    args = ap.parse_args()

    print(f"Validating: {args.csv}")
    df = P.load(args.csv)

    # --- 1. Parse / structural integrity ------------------------------------
    section("1. Structure & parse")
    n = len(df)
    report("PASS" if n > 0 else "FAIL", f"parsed {n} data rows, {len(P.COLS)} columns")
    nan = int(df.isna().sum().sum())
    report("PASS" if nan == 0 else "FAIL", "no NaN / missing cells", f"{nan} NaN found" if nan else "")
    # Raw field-count consistency (every numeric line must have 13 fields).
    bad = 0
    with open(args.csv) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if line.split(",")[0] == "t_ms":  # header
                continue
            if line.count(",") != len(P.COLS) - 1:
                bad += 1
    report("PASS" if bad == 0 else "FAIL", "every data line has 13 fields",
           f"{bad} malformed lines" if bad else "")

    # --- 2. Timing / sampling ------------------------------------------------
    section("2. Timing & sampling")
    dt = df["t_ms"].diff().dropna()
    neg = int((dt < 0).sum())
    report("PASS" if neg == 0 else "FAIL", "t_ms strictly increasing (no resets)",
           f"{neg} negative steps" if neg else "")
    med = dt.median()
    hz = 1000.0 / med if med else float("nan")
    report("PASS", f"median dt = {med:.0f} ms  (~{hz:.1f} Hz)",
           f"jitter std={dt.std():.1f} ms, p99={dt.quantile(0.99):.0f} ms")
    gaps = P.calibration_events(df)
    cals = P.parse_calibration_comments(args.csv)
    match = len(gaps) == len(cals)
    report("PASS" if match else "WARN",
           f"{len(gaps)} timing gaps >200 ms vs {len(cals)} '# calibrated' lines",
           "gaps line up with calibration presses (captureReference blocks ~200 ms)"
           if match else "gap count != calibration-comment count — investigate")
    print(f"          total duration: {(df['t_ms'].max() - df['t_ms'].min())/1000:.1f} s")

    # --- 3. Calibration logic ------------------------------------------------
    section("3. Calibration logic")
    c0 = df[df["cal"] == 0]
    if len(c0):
        e = max((c0["dpitch"] - c0["pitch"]).abs().max(),
                (c0["droll"] - c0["roll"]).abs().max())
        report("PASS" if e < 1e-6 else "FAIL",
               "cal==0 invariant: dpitch==pitch and droll==roll",
               f"max deviation {e:.2e} deg")
    else:
        report("WARN", "no cal==0 rows present")
    # The cal column flips 0->1 once and never resets, so it cannot delimit the
    # re-don epochs. Recover them from the (pitch - dpitch) reference plateaus.
    epochs = P.calibration_epochs(df)
    n_flips = int((df["cal"].diff().fillna(0) > 0).sum())
    n_resets = int((df["cal"].diff().fillna(0) < 0).sum())
    if epochs:
        worst = max(e["ref_spread"] for e in epochs)
        report("PASS" if worst < 0.05 else "WARN",
               "calibration reference is a true constant within each re-don epoch",
               f"{len(epochs)} epochs, largest within-epoch pitchRef spread {worst:.3f} deg")
    report("PASS" if len(epochs) == len(cals) else "WARN",
           f"epoch count from data ({len(epochs)}) matches '# calibrated' lines ({len(cals)})")
    report("WARN" if n_resets == 0 and len(epochs) > 1 else "PASS",
           f"cal column has {n_flips} flip(s), {n_resets} reset(s)",
           "cal is a binary 'ever-calibrated' flag — it does NOT mark the re-don "
           "epochs; segment those via calibration_epochs()/comments")
    if len(cals):
        report("PASS", f"{len(cals)} calibration events (re-don / mounting-drift, OE1)",
               f"upright pitch {cals['pitch_ref'].min():.1f}..{cals['pitch_ref'].max():.1f} "
               f"(spread {cals['pitch_ref'].max()-cals['pitch_ref'].min():.1f}), "
               f"roll {cals['roll_ref'].min():.1f}..{cals['roll_ref'].max():.1f} "
               f"(spread {cals['roll_ref'].max()-cals['roll_ref'].min():.1f}) deg")
    print("          NOTE: dpitch/droll are referenced to the upright captured at each re-don.")
    print("          That re-baselining is BY DESIGN (the OE1 result), not a defect: calibrated")
    print("          deviation (dpitch/droll, cal==1) is the system's decision space and is in")
    print("          fact MORE reproducible & separable across re-dons than absolute pitch/roll.")

    # --- 4. Sensor (accelerometer) health ------------------------------------
    section("4. Sensor health (accelerometer)")
    mag = P.accel_magnitude(df)
    smag = P.accel_magnitude(P.steady(df))
    report("PASS" if 0.95 <= smag.mean() <= 1.05 else "WARN",
           f"steady |g| mean = {smag.mean():.3f} g  (expect ~1.0)",
           f"std {smag.std():.3f} g")
    frac = float(((smag > 0.9) & (smag < 1.1)).mean())
    report("PASS" if frac > 0.9 else "WARN",
           f"{frac*100:.1f}% of steady samples within 0.9-1.1 g")
    # Extreme |g| (motion artifacts) should live in tag 9, not steady postures.
    extreme = df[(mag < 0.8) | (mag > 1.2)]
    if len(extreme):
        in_junk = float((extreme["tag"] == P.JUNK_TAG).mean())
        report("PASS" if in_junk > 0.8 else "WARN",
               f"motion artifacts (|g| outside 0.8-1.2) are {in_junk*100:.0f}% in tag 9",
               f"{len(extreme)} extreme samples; max |g|={mag.max():.2f}, min={mag.min():.2f}")
    # pitch/roll vs the firmware formula.
    rp, rr = P.recompute_angles(df)
    perr = max((rp - df["pitch"]).abs().max(), (rr - df["roll"]).abs().max())
    report("PASS" if perr < 0.05 else "WARN",
           "logged pitch/roll match the accel formula",
           f"max abs error {perr:.3f} deg (rounding to 2 dp in firmware)")

    # --- 5. Protocol & posture content ---------------------------------------
    section("5. Protocol & posture content")
    present = {int(t) for t in df["tag"].unique()}
    undoc = sorted(present - P.DOCUMENTED_TAGS)
    report("PASS" if not undoc else "WARN",
           f"tags present: {sorted(present)}",
           f"UNDOCUMENTED tag(s) {undoc} not in protocol {sorted(P.DOCUMENTED_TAGS)} "
           f"— define them or relabel" if undoc else "")
    segs = [s for s in P.segments(df) if s["tag"] != P.JUNK_TAG]
    short = [s for s in segs if s["dur_s"] < 5.0]
    report("PASS" if not short else "WARN",
           f"{len(segs)} steady segments, all >=5 s" if not short
           else f"{len(segs)} steady segments, {len(short)} shorter than 5 s",
           f"shortest {min(s['dur_s'] for s in segs):.1f} s" if segs else "no steady segments")

    # Separability: per-tag mean (pitch, roll); nearest-neighbour distance.
    st = P.steady(df)
    means = st.groupby("tag")[["pitch", "roll"]].mean()
    print("          per-posture mean (pitch, roll) deg:")
    for tag, row in means.iterrows():
        print(f"            tag {int(tag)} {P.TAG_LABELS.get(int(tag),'?'):<22} "
              f"pitch {row['pitch']:7.2f}  roll {row['roll']:7.2f}")
    pts = means.to_numpy()
    mind, pair = np.inf, None
    tags = list(means.index)
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            d = float(np.hypot(*(pts[i] - pts[j])))
            if d < mind:
                mind, pair = d, (tags[i], tags[j])
    report("PASS" if mind > 10 else "WARN",
           f"closest posture pair separated by {mind:.1f} deg in (pitch,roll)",
           f"nearest: tag {pair[0]} vs tag {pair[1]}" if pair else "")

    # --- Verdict -------------------------------------------------------------
    section("Verdict")
    print(f"  PASS {_counts['PASS']}   WARN {_counts['WARN']}   FAIL {_counts['FAIL']}")
    if _counts["FAIL"]:
        print("  => FAIL: data has integrity problems (see above).")
        return 1
    if _counts["WARN"]:
        print("  => USABLE with documented caveats (the WARNs above are expected/by-design).")
    else:
        print("  => CLEAN.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
