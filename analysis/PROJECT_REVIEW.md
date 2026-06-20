# Project Review — TCC Posture Monitor (Midterm / Entrega Parcial)

**Date:** 2026-06-20 (submission day) · **Scope:** existing data + analysis/writing ONLY (no new captures)
**Verdict:** **QUALIFIED GO.** The analysis is fundamentally trustworthy — ~90% of headline numbers reproduce exactly from raw data, the tooling is deterministic, and the project is unusually honest about its limits. Three doc-level defects MUST be fixed before submission; all are fixable tonight by editing prose / recomputing only. Frame the results as **motivating** the design, not **validating** an integrated system.

---

## 1. Overall assessment

| Channel | Status | One-line |
|---|---|---|
| **IMU (Campaign A)** | Trustworthy, 1 framing fix | 4/5 headline numbers exact; the OE1 self-correction in VALIDATION.md is wrong and undersells the result. |
| **FSR (Campaign B)** | Trustworthy, 2 must-fixes | Most numbers exact incl. the strongest result (B6 11.5x); B5 creep and part-to-part @1kg are wrong/cherry-picked. |
| **Code quality** | Good, 1 critical + 1 major bug | All plot scripts run; `split_direction` peak-dwell bug kills the validate_fsr seating gate. No published FSR number is wrong because of it. |

**Bottom line:** midterm-ready as *descriptive single-channel feasibility*. NOT ready as "integrated system validation." Fix the three P0 numbers, reframe OE2/OE4, and submit.

---

## 2. Corrected numbers (fix before submitting)

| What | Doc claims | Correct (recomputed from raw) | Files |
|---|---|---|---|
| **B5 FSR2 creep** | ~2 min, **-5.1%** (253→240 Ω); "just early drift" | **12.0 min, -11.5%** (253→224 Ω, -0.96 %/min). FSR2 creeps MORE than FSR1 (-7.4%). The 240/-5.1% is exactly the 2-min-bin value. | `data/README.md:70`, `analysis/FSR_FINDINGS.md:17` |
| **Part-to-part @1 kg** | **~5%** (293/280/283 Ω) | **~39–46%** with a consistent rule (overall 290/267/391). 283 is FSR3's anomalous up-sweep; FSR3 overall=391/down=398. | `data/README.md:87-88`, `analysis/FSR_FINDINGS.md:34` |
| **OE1 roll-drift caveat** | 21° roll "inflated by tag==9 junk; corrected above" | **FALSE.** Steady-hold (tag==0) drift = **8.0° pitch / 21.6° roll** — roll is LARGER. Dropping junk epochs moves it ~1°. | `analysis/VALIDATION.md:130-135` |
| **Re-don count (OE1 n)** | 7 re-don events | **n=4 genuine** re-dons (3 are tag==9 double-press junk). | `VALIDATION.md:22`, `postures.py:143-172` |
| **Bad-posture p05** | "every bad posture ≥ 18.6°" | 18.6 is the **pooled** p05; lowest per-tag = **18.0°** (tag3). 0%/100% @10° still holds. | `VALIDATION.md:108-109` |
| **Floor onset** | "by ≥1.6 kg, all three" | Only FSR2 near floor @1.6 kg; FSR1/FSR3 reach it @2 kg. Soften to **"~1.6–2 kg"**. Band 158–180 Ω is exact. | `data/README.md:80-82` |

### Disputes resolved in favor of the independent recompute
- **Bad p05 = 18.6 STANDS** (a reviewer claimed ~14–16; that used `cal==0` rows where Δpitch/Δroll are raw absolute angles, not the calibrated deviations the metric is defined on). On the `cal==1` decision space the doc actually uses, pooled bad p05 = **18.3** (≈18.6).
- **1↔4 gap = 6.9° STANDS** — verified 6.88° absolute AND 6.55° calibrated. The doc is correct in both spaces.
- **"~3 Ω/count" STANDS** — verified as the literal divider dR/draw at the floor (~3.1 Ω/count). The claim it was a "wording bug" did NOT survive verification.

---

## 3. Code bugs (surviving verification)

| Sev | File | Bug | Midterm impact |
|---|---|---|---|
| **Critical** | `validate_fsr.py:159-168` | Section-6 seating gate is **dead — always PASS**. `split_direction` on a single-load subset degenerates to up=1/down=N for every level; the `len(u)<5` guard skips all. Passes on the FSR2 50g ×4.2 / FSR3 200g ×3.1 transients it advertises catching. | No published number depends on it. Fix the one-time split, OR footnote that the WORKING seating flag is in `plot_fsr_parttopart.py`. |
| **Major** | `fsr_lib.py:156` | `split_direction` splits at `idxmax()` (first peak), mislabeling the 250–330-sample peak dwell as 'down' → top up-sweep rung = 1 unaveraged sample. Root cause of the critical bug. | Published ~180 Ω floor is unaffected (shared by up/down/all). Methodology defect only. |
| Minor | `postures.py` | `load()` crashes on pandas 3.0 at the `cal` int-cast on a NaN row. | Figures already exist; recompute from raw. |
| Minor | `plot_fsr_curve.py:46-50,68` | Bypasses jitter-merge (11 levels not 10); defaults `--units kg` on gram data (mislabels axis, breaks capped test). | Regenerate with `--units g`. |
| Minor | `validate_fsr.py:151` | Dead `capped` variable; PASS reported unconditionally. | Cosmetic. |

---

## 4. Midterm readiness

### DONE (with backing figure/file)
- **OE1 calibration justified:** IMU re-don drift 7.7° pitch / 21.6° roll, measured on the worn unit. `figures/04_redon_drift.png`
- **OE2 feasibility:** 5/6 postures separate cleanly; accel |g|=0.993±0.004 g. `figures/02_separation.png`
- **10° alert threshold:** upright p95=6.4°, pooled-bad p05=18.6°, 0%/100% @10°. `figures/06_alert_threshold.png`
- **1↔4 hard-pair (fusion motivation):** gap 6.9° vs ≥22° all other pairs. `figures/02_separation.png`
- **FSR part-to-part (N=3):** 457/529/502 @400g, 293/280/283 @1kg, 180/164/173 @2kg; floor 158–180 Ω. `analysis/figures/parttopart.png`
- **FSR3 B6 seating artifact (strongest FSR result):** 400g up 11.3k→8.6k (std 958) vs down 812 = 11.5×; validate_fsr WARNs 600g ×5.0. `analysis/figures/parttopart.png`
- **FSR model:** power-law R²~0.97, conductance-linear ~0.95 vs R-linear ~0.43; ~10 Ω/100g, ~1.7%/count. `analysis/figures/model.png`
- **FSR hysteresis & creep:** FSR2 -76%→-6%; FSR1 creep -7.4%/11 min. `analysis/figures/hysteresis.png`, `creep.png`

### Missing / impossible without new tests
- Sensor fusion (FSR + IMU together) — no fused capture exists
- Real chair-mounted FSR data — only rigid-puck bench data
- 2nd/multiple subjects; neck-mounted IMU — run 1 is N=1 bare XIAO
- Naturalistic (non-pronounced) posture margins
- Radio (ESB) / mux / battery / 2nd XIAO — not built
- Inferential statistics — not warranted at N=1 / N=3

### Salvageable by writing only (tonight)
All of Section 5 below.

---

## 5. Prioritized actions (ALL achievable tonight, no captures)

**P0 — must fix, examiner will catch by rerunning the tool**
1. **B5 creep** → 12 min, -11.5% (253→224); remove the "short / don't compare to B3" caveat. Re-run `plot_fsr_creep.py`, paste its line. *(~15 min)*
2. **VALIDATION.md:130-135** → replace the false "inflated by junk" note with steady-hold 8.0°/21.6°, n=4 re-dons. *(~15 min)*
3. **Part-to-part @1kg** → recompute all 3 sensors with ONE direction rule; report ~39–46%; keep "still ≪ 11× seating." *(~20 min)*

**P1 — framing, pre-empt the sharp banca questions**
4. Reframe **OE2/OE4** to "evidência para viabilidade"; note integration/fusion/efficiency + OE3 deferred; lead fusion motivation with the 1↔4 overlap. *(~25 min)*
5. Reframe calibration from "MANDATORY" → "strongly motivates"; label the 11× a bench rigid-puck transient; lead with the IMU re-don leg. *(~20 min)*
6. Add ONE "Limitações e validade estatística" subsection (N=1, N=3, single session, pronounced postures, bench≠chair, no fusion; no inferential stats claimed). *(~20 min)*
7. Footnote the 10° threshold: n=4 re-dons, present as starting point not performance; reword bad p05 to "pooled 18.6° / lowest 18.0° tag3." *(~15 min)*

**P2 — polish**
8. Soften "floor by ≥1.6 kg" → "~1.6–2 kg." *(~5 min)*
9. Frame relative-pressure/conductance choice as a principled FSR402-datasheet design decision. *(~10 min)*
10. (Code, optional) Note/fix the dead validate_fsr section-6 seating gate. *(~5–15 min)*

---

## 6. Examiner-risk table

| Risk (where the banca pushes) | Mitigation |
|---|---|
| "Where is the integrated system?" — OE2/OE4 mapped to single-channel bench data | Downgrade to "evidência/precursor"; defer integration+efficiency; lead OE2 with the 1↔4 IMU-overlap (IMU alone insufficient → fusion needed). |
| "Your 11× rests on one suspect up-sweep of one rigid puck — does the chair see it?" | Reframe to "strongly motivates"; label as bench install-sensitivity *in principle*; lead with the directly-measured IMU re-don drift. |
| "I recomputed part-to-part @1kg and got ~40%, not 5%." | Recompute with one consistent rule; report ~10–46%; conclusion survives (46% ≪ 11×). |
| "0%/100% depends on discarding a junk epoch + pronounced N=1 postures." | Footnote n=4 re-dons; present 10° as a starting point, not a performance figure; keep the optimistic-margin caveat prominent. |
| "N=1, N=3, single session — statistical validity?" | One consolidated Limitations paragraph; state no inferential stats are claimed at this N (descriptive feasibility by design). |
| "OE3 is missing from the table." | Add a one-line deferred-scope note. |

---

## 7. Strongest honest framing

> We independently bench-characterized each sensing channel on existing hardware and found that **both channels carry usable posture signal, but the dominant error source in each is install/mounting offset, not sensor quality** — the empirical motivation for per-user calibration and for sensor fusion, both deferred to the next phase. Re-mounting the IMU drifts the "upright" reference by **8.0° pitch / 21.6° roll** (→ fixed thresholds misfire → calibrate per user); the worn IMU cleanly separates 5 of 6 postures but **cannot resolve slouch from head-down** (gap 6.9° vs ≥22°) — exactly why the chair FSRs are needed. On the FSR side, absolute readings are install-sensitive in principle (an unbedded sensor reads ~11× high until it settles) while true part-to-part variation is far smaller, reinforcing that the chair channel should be read as **relative contact/distribution, not calibrated force** — grounded in the FSR402 saturation physics. These are descriptive single-channel feasibility results (N=1 subject, N=3 sensors) that **motivate the fused design; they do not yet validate the integrated system**, which is the explicit work of the next phase.
