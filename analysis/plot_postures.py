#!/usr/bin/env python3
"""Generate the figures for the IMU posture capture (postures.csv).

Writes PNGs to analysis/figures/:
  01_timeline.png          pitch & roll over the whole capture, shaded by posture,
                           calibration presses marked
  02_separation.png        pitch-vs-roll scatter — do the postures separate? (OE2/OE4)
  03_distributions.png     per-posture pitch & roll box plots
  04_redon_drift.png       upright reference drift across re-don calibrations (OE1)
  05_sensor_health.png     accel |g| over time + histogram (steady vs motion)

Usage:
    .venv/bin/python analysis/plot_postures.py [--csv postures.csv] [--show]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless; --show switches to an interactive backend
import matplotlib.pyplot as plt
import numpy as np

import postures as P

FIG_DIR = Path(__file__).resolve().parent / "figures"


def _label(tag: int) -> str:
    return P.TAG_LABELS.get(tag, f"tag {tag}")


def _shade_postures(ax, df):
    """Shade the time axis behind each held (non-junk) posture segment."""
    seen = set()
    for s in P.segments(df):
        if s["tag"] == P.JUNK_TAG:
            continue
        ax.axvspan(s["t0"] / 1000, s["t1"] / 1000,
                   color=P.TAG_COLORS.get(s["tag"], "#999"), alpha=0.18,
                   label=_label(s["tag"]) if s["tag"] not in seen else None)
        seen.add(s["tag"])


def fig_timeline(df, cals_gaps):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    t = df["t_ms"] / 1000
    a1.plot(t, df["pitch"], lw=0.7, color="#222")
    a2.plot(t, df["roll"], lw=0.7, color="#222")
    for ax, name in ((a1, "pitch (forward/back lean)"), (a2, "roll (left/right lean)")):
        _shade_postures(ax, df)
        for _, g in cals_gaps.iterrows():
            ax.axvline(g["t_ms"] / 1000, color="k", ls="--", lw=0.8, alpha=0.6)
        ax.set_ylabel(f"{name}  [deg]")
        ax.grid(alpha=0.3)
    # de-dup legend
    h, l = a1.get_legend_handles_labels()
    seen, hh, ll = set(), [], []
    for hi, li in zip(h, l):
        if li not in seen:
            seen.add(li); hh.append(hi); ll.append(li)
    a1.legend(hh, ll, ncol=6, fontsize=8, loc="upper center", bbox_to_anchor=(0.5, 1.18))
    a2.set_xlabel("time [s]")
    a1.set_title("IMU posture capture — full timeline "
                 "(shaded = held posture, dashed = calibration press)", pad=28)
    fig.tight_layout()
    return fig


def _scatter_postures(ax, data, xcol, ycol):
    for tag, g in data.groupby("tag"):
        c = P.TAG_COLORS.get(tag, "#999")
        ax.scatter(g[xcol], g[ycol], s=6, alpha=0.22, color=c)
        mx, my = g[xcol].mean(), g[ycol].mean()
        ax.scatter([mx], [my], s=170, color=c, edgecolor="k", zorder=5,
                   label=f"{_label(tag)}  ({mx:.0f}, {my:.0f})")
        ax.errorbar(mx, my, xerr=g[xcol].std(), yerr=g[ycol].std(),
                    color="k", alpha=0.4, capsize=3, zorder=4)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7.5, title="posture (mean x, y)")


def fig_separation(df):
    """Left: raw absolute angles (all rounds) — clusters split by re-don drift.
    Right: calibrated deviation (cal==1) — the system's actual decision space."""
    st = P.steady(df)
    st_cal = st[st["cal"] == 1]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(15, 7))
    _scatter_postures(a1, st, "pitch", "roll")
    a1.set_xlabel("pitch [deg]  (forward/back)")
    a1.set_ylabel("roll [deg]  (left/right)")
    a1.set_title("Absolute angles, all rounds\n(sub-clusters = re-don mounting drift)")
    _scatter_postures(a2, st_cal, "dpitch", "droll")
    a2.set_xlabel("dpitch [deg]  (deviation from upright)")
    a2.set_ylabel("droll [deg]")
    a2.set_title("Calibrated deviation, cal==1\n(decision space — tighter, more separable)")
    fig.suptitle("Posture separability — markers = per-posture mean ±1σ "
                 "(forward-slouch ↔ head-down is the one hard pair)")
    fig.tight_layout()
    return fig


def fig_distributions(df):
    st = P.steady(df)
    tags = sorted(st["tag"].unique())
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 6))
    for ax, col, name in ((a1, "pitch", "pitch [deg]"), (a2, "roll", "roll [deg]")):
        data = [st[st["tag"] == t][col].to_numpy() for t in tags]
        bp = ax.boxplot(data, patch_artist=True, showfliers=False,
                        tick_labels=[_label(t) for t in tags])
        for patch, t in zip(bp["boxes"], tags):
            patch.set_facecolor(P.TAG_COLORS.get(t, "#999"))
            patch.set_alpha(0.65)
        for med in bp["medians"]:
            med.set_color("k")
        ax.set_ylabel(name)
        ax.set_title(f"{name} by posture")
        ax.grid(alpha=0.3, axis="y")
        ax.tick_params(axis="x", rotation=30)
    fig.suptitle("Per-posture angle distributions (transitions excluded)")
    fig.tight_layout()
    return fig


def fig_redon(cals):
    fig, ax = plt.subplots(figsize=(9, 5.5))
    x = cals["event"] + 1
    ax.plot(x, cals["pitch_ref"], "o-", color="#1f77b4", label="upright pitch ref")
    ax.plot(x, cals["roll_ref"], "s-", color="#d62728", label="upright roll ref")
    for _, r in cals.iterrows():
        ax.annotate(f"{r['pitch_ref']:.1f}", (r["event"] + 1, r["pitch_ref"]),
                    textcoords="offset points", xytext=(0, 8), fontsize=8, ha="center")
        ax.annotate(f"{r['roll_ref']:.1f}", (r["event"] + 1, r["roll_ref"]),
                    textcoords="offset points", xytext=(0, -14), fontsize=8, ha="center")
    pr = cals["pitch_ref"]; rr = cals["roll_ref"]
    ax.set_xlabel("re-don / calibration event #")
    ax.set_ylabel("captured 'upright' reference [deg]")
    ax.set_title("Re-don drift of the 'upright' reference\n"
                 f"pitch spread {pr.max()-pr.min():.1f} deg, "
                 f"roll spread {rr.max()-rr.min():.1f} deg "
                 "— fixed thresholds would misfire (motivates calibration)")
    ax.set_xticks(list(x))
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    return fig


def fig_sensor_health(df):
    mag = P.accel_magnitude(df)
    junk = df["tag"] == P.JUNK_TAG
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5), gridspec_kw={"width_ratios": [3, 1]})
    t = df["t_ms"] / 1000
    a1.scatter(t[~junk], mag[~junk], s=3, alpha=0.3, color="#2ca02c", label="steady posture")
    a1.scatter(t[junk], mag[junk], s=3, alpha=0.3, color="#cfcfcf", label="transition (tag 9)")
    a1.axhline(1.0, color="k", lw=1, ls="--", label="1 g (rest)")
    a1.set_xlabel("time [s]"); a1.set_ylabel("|accel| [g]")
    a1.set_title("Accelerometer magnitude over time")
    a1.set_ylim(0, min(3, mag.max() * 1.05)); a1.grid(alpha=0.3); a1.legend(fontsize=8)
    a2.hist(mag[~junk], bins=60, range=(0.5, 1.5), orientation="horizontal",
            color="#2ca02c", alpha=0.7, label="steady")
    a2.hist(mag[junk], bins=60, range=(0.5, 1.5), orientation="horizontal",
            color="#cfcfcf", alpha=0.6, label="tag 9")
    a2.axhline(1.0, color="k", lw=1, ls="--")
    a2.set_xlabel("count"); a2.set_title("distribution")
    a2.set_ylim(0.5, 1.5); a2.legend(fontsize=8)
    fig.suptitle(f"Sensor health — steady |g| mean = {P.accel_magnitude(P.steady(df)).mean():.3f} g")
    fig.tight_layout()
    return fig


def fig_alert(df, threshold=10.0):
    """Deviation metric sqrt(dpitch^2+droll^2) per posture (cal==1) and a
    candidate alert angle separating upright from held bad postures."""
    c = P.steady(df)
    c = c[c["cal"] == 1].copy()
    if c.empty:
        return None
    c["dev"] = P.deviation(c)
    tags = sorted(c["tag"].unique())
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5.5), gridspec_kw={"width_ratios": [1, 1.3]})

    up = c[c["tag"] == 0]["dev"]
    bad = c[c["tag"] != 0]["dev"]
    a1.hist(up, bins=40, range=(0, 45), color=P.TAG_COLORS[0], alpha=0.8, label="upright (tag 0)")
    a1.hist(bad, bins=40, range=(0, 45), color="#d62728", alpha=0.5, label="bad postures (1-5)")
    a1.axvline(threshold, color="k", ls="--", lw=1.5, label=f"alert @ {threshold:.0f}°")
    a1.set_xlabel("deviation from upright  √(Δpitch²+Δroll²) [deg]")
    a1.set_ylabel("count"); a1.set_title("Upright vs bad-posture deviation")
    a1.legend(fontsize=8); a1.grid(alpha=0.3)

    data = [c[c["tag"] == t]["dev"].to_numpy() for t in tags]
    bp = a2.boxplot(data, orientation="horizontal", patch_artist=True, showfliers=False,
                    tick_labels=[_label(t) for t in tags])
    for patch, t in zip(bp["boxes"], tags):
        patch.set_facecolor(P.TAG_COLORS.get(t, "#999")); patch.set_alpha(0.65)
    for med in bp["medians"]:
        med.set_color("k")
    a2.axvline(threshold, color="k", ls="--", lw=1.5)
    a2.set_xlabel("deviation from upright [deg]")
    a2.set_title(f"Per-posture deviation (cal==1) — alert @ {threshold:.0f}°")
    a2.grid(alpha=0.3, axis="x")
    fp = float((up > threshold).mean()) * 100
    tp = float((bad > threshold).mean()) * 100
    fig.suptitle(f"Proposed alert angle ≈ {threshold:.0f}°  →  upright false-alarm {fp:.1f}%, "
                 f"bad-posture caught {tp:.0f}%  (margin is wide here because postures were pronounced)")
    fig.tight_layout()
    return fig


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", default=str(P.DEFAULT_CSV))
    ap.add_argument("--show", action="store_true", help="display windows instead of only saving")
    args = ap.parse_args()
    if args.show:
        matplotlib.use("TkAgg", force=True)

    df = P.load(args.csv)
    cals = P.parse_calibration_comments(args.csv)
    gaps = P.calibration_events(df)
    FIG_DIR.mkdir(exist_ok=True)

    figs = {
        "01_timeline.png": fig_timeline(df, gaps),
        "02_separation.png": fig_separation(df),
        "03_distributions.png": fig_distributions(df),
        "05_sensor_health.png": fig_sensor_health(df),
    }
    if len(cals):
        figs["04_redon_drift.png"] = fig_redon(cals)
    else:
        print("note: no '# calibrated upright' lines found — skipping re-don figure")
    alert = fig_alert(df)
    if alert is not None:
        figs["06_alert_threshold.png"] = alert
    else:
        print("note: no calibrated (cal==1) data — skipping alert-threshold figure")

    for name, fig in figs.items():
        out = FIG_DIR / name
        fig.savefig(out, dpi=130)
        print(f"wrote {out}")
    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
