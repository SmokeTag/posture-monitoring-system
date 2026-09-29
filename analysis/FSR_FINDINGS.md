# FSR402 data — evaluation & what was under-explored (Campaign B)

> Companion to `VALIDATION.md` (IMU). Written 2026-06-19 after a multi-lens review
> of every `data/*fsr*.csv`. All numbers below were recomputed from the raw CSVs
> via `fsr_lib.py`; the headline correction (FSR3 light end) was confirmed by an
> adversarial re-check. Feeds the partial report's *Resultados parciais* (OE2/OE4)
> and the calibration argument (OE1).

## 1. What we have

| run | sensor | kind | `validate_fsr.py` | one-line |
|----:|--------|------|-------------------|----------|
| B1 | FSR1 | sweep  | usable (caveats) | long/messy, 18 fragmented levels |
| B2 | FSR1 | sweep2 | usable (caveats) | clean ladder 50→2002 g, R 4134→180 Ω |
| B3 | FSR1 | creep  | CLEAN | 11 min @1 kg, −7.4 % drift |
| B4 | FSR2 | sweep  | usable (caveats) | 50→2000 g, R 3034→158 Ω |
| B5 | FSR2 | creep  | PASS (8/1/0) | ~12 min @1 kg, **−11.5 %** (drifts *more* than FSR1) |
| B6 | FSR3 | sweep  | usable (caveats) | recapture of the old VOID; light end seating-compromised |
| B7 | FSR3 | sweep2 | **CLEAN (13/0/0)** | well-seated 200→2001 g; the N=3 point |

"caveats" = the expected light-end hysteresis/seating WARNs, not integrity FAILs.

## 2. Already analysed before this review

- **R-vs-load curve + saturation knee** (`plot_fsr_curve.py`).
- **Creep** under a constant 1 kg load (`plot_fsr_creep.py`).
- Prose cross-run notes in `data/README.md` (floor, repeatability, part-to-part,
  hysteresis) — but with **no figure** for part-to-part or hysteresis and **no
  model fit**.

## 3. What was under-explored — now built

1. **Part-to-part as a *figure*, N=3** (`plot_fsr_parttopart.py --both`). Using a
   consistent all-samples median per level: 400 g = 453/514/502 Ω (FSR1/2/3, 13 %),
   1.6 kg = 192/158/183 (22 %), 2 kg = 180/164/173 (10 %); intermediate 1 kg is
   noisier at 290/267/391 (**~46 %**, FSR3-driven). All converge to a shared
   ~160–180 Ω floor. (The earlier "5 % at 1 kg" used FSR3's low *up-sweep* point
   283 Ω vs its 391 Ω overall — a per-sensor cherry-pick; quote one direction rule.)
2. **Hysteresis loop as a *figure*** (`plot_fsr_hysteresis.py`). FSR2 −76 % @50 g →
   −6 % @1 kg; mean |hyst| ≤400 g ≈ 52 %, ≥1 kg ≈ 6 %. The operating regime is
   direction-independent.
3. **Response model** (`plot_fsr_model.py`). Power law R = a·F^b fits R²≈0.95–0.97,
   **but conductance G = 1/R is ~linear in force** (R²≈0.95 vs R-linear 0.43) →
   conductance is the right variable for a relative-pressure map.
4. **Resolution / divider** (in `plot_fsr_model.py`). Operating sensitivity
   ≈10 Ω/100 g; the 10-bit ADC gives ~3 Ω/count at the floor ≈ **1.7 %/count** →
   only a handful of resolvable pressure bands when seated.
5. **A capture gate** (`validate_fsr.py`) that would have auto-caught the original
   VOID *and* the B6 seating transient — turn-key for run 3 and every future run.

## 4. The correction the adversarial re-check forced

The first read of B6 (FSR3) showed ~9.3 kΩ at 400 g — a ~20× "part-to-part" gap.
**That is a seating artifact, not part spread.** On the up-sweep the puck had not
yet bedded into the pad: at 400 g it reads 11.3 kΩ *settling to* 8.6 kΩ (std 958),
while the *down*-sweep at the same 400 g is a rock-steady **812 Ω** — an 11×
up/down ratio (a healthy FSR is ~1.25×). So:

> **Seating/mounting variation (3–11×) dwarfs true part-to-part variation (≤40 %).**

This is the **stronger** result for the thesis: how a sensor is *seated* swings its
absolute reading by an order of magnitude, so a fixed absolute threshold is fragile
→ this **strongly motivates per-install / per-user calibration** (OE1; reinforces
`docs/decisao-calibracao-vs-limiar-absoluto.md`). **Caveat for the report:** this
3–11× is a *bench up-sweep transient on an unbedded rigid puck*, i.e. evidence of
install-sensitivity *in principle* — the real swing on a foam-mounted chair FSR
under a human is still to be measured (Próximas etapas). The directly-measured leg
is the IMU re-don drift (see `VALIDATION.md`). The figures flag seating-suspect
points (hollow markers) and exclude them from the spread.

## 5. Honest caveats for the written report

- The **~160–180 Ω "floor" is the reading at the 2 kg scale cap**, not the true
  asymptote — the curve is still descending there (FSR1 −6 % on the last step).
  State saturation as *"R is already low and flattening by 2 kg, and a seated adult
  loads well beyond the scale"* — do **not** quote a calibrated floor.
- The pronounced, deliberate loads make separation **optimistic** (same caveat as
  Campaign A's postures).

## 6. Still not explored (next steps / *Próximas etapas*)

- **Release/recovery run** — creep measured loading drift but not how fast R
  *recovers* after unload; matters for a continuously-sampling alert (does the
  reading reset between posture changes?).
- **Lower `R_FIXED` (~1 kΩ) + a bigger scale** — to reach the true seated floor
  beyond the 2 kg cap and confirm the asymptote.
- **FSR + IMU together** — the project's novelty (vs BackWatch) is fusing chair
  pressure with the worn IMU; no joint/contemporaneous capture exists yet.
- **Uncontrolled temperature / time-of-day** — FSRs drift with both; note as a
  limitation, or hold conditions in a future run.
