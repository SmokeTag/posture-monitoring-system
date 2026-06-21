# Validation report — `postures.csv`

**Verdict: USABLE — structurally clean, with documented caveats.**
Checks: **15 PASS / 3 WARN / 0 FAIL**. The data is safe to use as-is for the
midterm OE1/OE2/OE4 analysis; the WARNs are expected/by-design behaviour, not
data defects.

Source: `data/2026-06-19_andre_no-neck-mount_own-chair.csv` — serial capture
from `firmware/xiao_imu_test/xiao_imu_test.ino` (XIAO nRF52840 Sense). **Run 1:
André Thomas Monteiro, no neck mount (bare XIAO), his own office chair.** Re-run
the checks with `.venv/bin/python analysis/validate_postures.py`.

## What the file contains

| | |
|---|---|
| Data rows | 17 937 (+ 70 `#` comment lines = 18 007 total) |
| Duration | 964.9 s (≈16.1 min), `t_ms` 32 389 → 997 259 |
| Sample rate | median dt = 54 ms (~18.5 Hz); 99.96 % of intervals in [45, 65] ms |
| Structure | **5 full rounds** of the posture sequence `0→1→2→3→4→5`, transitions tagged `9` between every hold |
| Holds | 31 steady segments, all ≥ 9.1 s (29/31 ≥ 10 s) |
| Calibration | **7 calibration presses**, of which **4 are genuine re-don epochs** with a held upright (the other 3 were immediate double-presses that captured only `tag==9` junk) — the OE1 experiment (honest **n = 4** re-dons) |
| Tags present | 0 upright, 1 fwd-slouch, 2 lean-L, 3 lean-R, 4 head-down, **5 laid-back**, 9 transition  (counts: 1427/1222/1111/1435/1390/1286/10066) |

## Checks

### 1. Structure & parse — PASS
Every data row has exactly 13 comma-separated fields; `pd.read_csv(comment='#')`
recovers all 17 937 numeric rows with **zero loss and zero NaN**. Dtypes correct
(3 int + 10 float), ranges sane, line endings uniform CRLF, `t_ms` strictly
increasing (no `millis()` reset, no duplicates).

> ⚠️ **No header line in the file.** The first line is already data
> (`32389,9,0,…`). The provided loader uses `header=None` and is correct, but any
> alternative reader using `header=0`/`skiprows=1` would silently drop the first
> real sample. Keep readers header-agnostic.

### 2. Timing & sampling — PASS
Strictly monotonic over the whole record. The 54 ms-vs-50 ms-nominal offset is
the documented read overhead, not instability. **All 7 gaps > 200 ms are
268–269 ms and line up 1:1 with the 7 calibration events** (`captureReference()`
blocks ~200 ms); every gap falls inside a `tag==9` window that analysis drops
anyway, so **no posture data is lost**.

### 3. Calibration logic — PASS (+1 WARN)
- The firmware invariant holds **exactly**: for all 9 326 `cal==0` rows,
  `dpitch==pitch` and `droll==roll` (max deviation 0.0°).
- The 7 re-don references are each a **true constant** within their epoch
  (largest within-epoch `pitchRef` spread 0.010°), and the 7 data-derived epochs
  match the 7 `# calibrated upright` comment lines.

> ⚠️ **The `cal` column cannot delimit the re-don epochs.** It flips `0→1` once
> and never resets (no `'r'` was pressed), so all 6 re-calibrations happen with
> `cal` already 1. Segment re-dons from the calibration comments or the
> `pitch − dpitch` plateaus (`postures.calibration_epochs()`), not from `cal`.

### 4. Sensor health — PASS (+2 WARN, both minor)
- **Accelerometer is healthy:** steady `|g| = 0.993 ± 0.004 g`; 99.9 % of steady
  samples in 0.9–1.1 g. Every out-of-band `|g|` (down to 0.32 g, up to 6.10 g)
  is a `tag==9` motion sample — direct confirmation the motion tagging works.
- Logged pitch/roll reproduce the firmware `atan2` formulas to < 0.05° (pitch).

> ⚠️ Roll reconstruction differs by up to 0.32° on 554 rows — purely from accel
> being logged to 3 decimals combined with `atan2` sensitivity at small `|az|`.
> Negligible (postures differ by many degrees). Not a sensor/formula error.
>
> ⚠️ Gyro has a persistent zero-rate bias (gY ≈ −3.6°/s, gZ ≈ +1.0°/s). It does
> **not** affect posture: pitch/roll are accel-only. Subtract it as a constant
> *only if* you ever integrate the gyro.

### 5. Protocol & posture content — PASS (+1 WARN)

**`tag 5` = "laid-back"** (resolved). 1 286 rows (7.2 %), one tight
hold per round in all 5 rounds (within-hold std pitch ±0.43° / roll ±0.81°) — a
deliberate, repeated pose: hips slid forward, torso leaning **back** against the
chair, so it has the **highest pitch of any tag (63.8°)** (this mounting reads
backward lean as *high* pitch, forward lean as *low*). It is now in the tag map
(`firmware/xiao_imu_test/xiao_imu_test.ino`, `data/README.md`,
`analysis/postures.py`). Being a backward lean, it separates cleanly from the
forward postures.

> ⚠️ **Forward-slouch (1) and head-down (4) overlap** — centroid gap only ~6.9°
> in (pitch, roll), ~3× tighter than any other pair (next-worst ≈ 22–24°). Holds
> even in calibrated deviation space (1-vs-4 nearest-centroid accuracy ≈ 76 %, vs
> clean separation for every other pair). Physically expected (both bend the
> trunk/neck forward). **Important limitation:** this run's separation is
> *optimistic* — posture 1 was performed very pronounced. A subtler, more natural
> slouch would collapse the 1↔4 gap further. **Conclusion: postures 1 and 4 need
> the chair FSRs to be reliably classified** (the worn IMU alone is insufficient
> for this pair) — concrete motivation for the planned sensor fusion.

**The other 4 postures (2, 3, 5 + upright) separate cleanly** in (pitch, roll) —
the OE2/OE4 feasibility result. See `figures/02_separation.png` and
`figures/03_distributions.png`.

## Headline results for the report

- **OE1 (anatomical variation / calibration justified):** re-mounting drifts the
  captured "upright" reference by **8.0° in pitch and ~21.6° in roll** (recomputed
  from the steady `tag==0` holds, across the **4 genuine** re-don epochs) → a single
  fixed threshold would misfire → per-user calibration is justified.
  `figures/04_redon_drift.png`.
- **OE2/OE4 (sensor layer works / feasibility):** the IMU separates 5 of 6
  postures cleanly; the worn sensor measurably distinguishes upper-back posture.
  Calibrated deviation (`dpitch/droll`) is the system's decision space and is
  **more reproducible and more separable** across re-dons than absolute angles.
- **Proposed alert angle ≈ 10°:** the deviation metric `√(Δpitch²+Δroll²)` from
  the calibrated upright is ≤ 6.4° (p95) when sitting upright; the **pooled**
  bad-posture p05 is 18.6° (the lowest *single* posture is 18.0°, tag 3 / lean-
  right). A 10° alert threshold separates them with **0 % false alarms and 100 %
  of these bad postures caught**. `figures/06_alert_threshold.png`.
  ⚠️ Caveat: this margin is wide *because the postures were performed pronounced*
  — expect it to shrink under naturalistic posture and across people; treat 10°
  as a starting point to re-tune, not a final value.

## Action items

1. ~~Define `tag 5`~~ **done** — "laid-back", added to the tag map
   (firmware, plan doc, `postures.py`).
2. Treat **1 ↔ 4 (slouch ↔ head-down)** as a hard pair for the IMU channel; lean
   on chair pressure (FSRs) to separate them — motivates the sensor fusion.
3. **Re-capture with subtler / more naturalistic postures** to measure the true
   (non-exaggerated) separation margin — the optimistic case is now established;
   the realistic case is the open question.
4. Repeat with the **neck mount** and **other subjects** (this run: 1 subject,
   bare XIAO). See `docs/protocolo-captura-imu.md` for the repeatable procedure.
5. *(Optional firmware nicety)* emit an incrementing calibration id (or a
   momentary `cal=0`) on each `'c'` so re-don epochs are readable from a column.

---

*Method:* validated across 5 independent dimensions with adversarial
re-derivation of every material finding. One finding initially raised as
"critical" — *that `dpitch/droll` can't be pooled across re-don epochs* — was
**refuted** on re-derivation: calibrated deviation is empirically the *better*
representation. A later re-check also **corrected an earlier note** here that had
called the ~21° roll drift "inflated by junk": recomputing the drift from the
steady `tag==0` holds only (dropping the 3 immediate-double-press epochs) gives
**8.0° pitch / 21.6° roll** across the 4 genuine re-dons — the roll is, if
anything, slightly *larger*, not inflated. The OE1 result stands and is reported
above with the honest n = 4.
