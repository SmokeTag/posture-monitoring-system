# Capture data — IMU posture runs

Raw serial captures from `firmware/xiao_imu_test/xiao_imu_test.ino`. One CSV per
capture run. The procedure that produces these files:
[`../docs/protocolo-captura-imu.md`](../docs/protocolo-captura-imu.md).

## File naming

```
YYYY-MM-DD_<subject>_<mount>_<chair>.csv
```

- `subject` — short name (e.g. `andre`)
- `mount` — `no-neck-mount` (bare XIAO strapped on) or `neck-mount`
- `chair` — `own-chair`, `office-chair-A`, …
- Add `-run2`, `-run3` if more than one capture in a day with the same setup.

The analysis scripts default to the **newest** file here (ISO dates sort
chronologically); override with `--csv data/<file>.csv`.

## Schema

`t_ms,tag,cal,pitch,roll,dpitch,droll,ax,ay,az,gx,gy,gz` — see
[`../analysis/postures.py`](../analysis/postures.py) for full column docs.

Tag map: `0` upright · `1` forward slouch · `2` lean left · `3` lean right ·
`4` head-down · `5` laid-back · `9` transition/moving (dropped in
analysis). `#`-prefixed lines are comments (calibration events, tag stamps).

## Run registry

| run | file | date | subject | mount | chair | notes |
|----:|------|------|---------|-------|-------|-------|
| 1 | `2026-06-19_andre_no-neck-mount_own-chair.csv` | 2026-06-19 | André Thomas Monteiro | none (bare XIAO) | own office chair | 6 postures × 5 rounds + 7 re-don calibrations. Validated **15 PASS / 3 WARN / 0 FAIL** (`../analysis/VALIDATION.md`). Postures performed *pronounced* → separation is optimistic. First calibration was an immediate double-press. |

# Capture data — FSR402 characterization (Campaign B)

Raw serial captures from `firmware/uno_fsr_test/uno_fsr_test.ino` (Arduino Uno,
single FSR402 on a voltage divider). Procedure:
[`../docs/protocolo-fsr-balanca-precisao.md`](../docs/protocolo-fsr-balanca-precisao.md)
(precision-balance light-contact variant).

## File naming

```
YYYY-MM-DD_<sensor>_<subject>_<rfixed-ohms>_<kind>.csv
```

- `sensor` — which physical FSR402 part: `fsr` (unit 1), `fsr2`, `fsr3`, … —
  distinct sensors, for part-to-part spread.
- `rfixed-ohms` — measured divider resistor (e.g. `2833` = 2.833 kΩ measured).
- `kind` — `sweep` / `sweep2…` (R-vs-load up+down) or `creepNkg` (R-vs-time at a
  constant N-kg dead load).

## Schema

`t_ms,load_kg,raw,ohms` — `ohms = -1` means open / no force. **Units caveat:**
the column is named `load_kg` but the precision-balance runs stamp it in
**GRAMS** (`50`, `100`, … `2000`). The analysis scripts take `--units g` to
normalize. `#`-prefixed lines are comments (the CSV header, tag stamps).

## Run registry

| run | file | date | subject | R_fixed | kind | units | notes |
|----:|------|------|---------|--------:|------|-------|-------|
| B1 | `2026-06-19_fsr_andre_2833_sweep.csv` | 2026-06-19 | André Thomas Monteiro | 2833 Ω (measured) | sweep (up+down) | grams | 18 load levels 10→2004 g, ~20k loaded samples. R 4467→173 Ω; near-flat by ~1.2–1.6 kg (knee within the 2 kg cap — full saturation needs a bigger scale). |
| B2 | `2026-06-19_fsr_andre_2833_sweep2.csv` | 2026-06-19 | André Thomas Monteiro | 2833 Ω (measured) | sweep2 (repeat) | grams | Clean ladder 50→2002 g, ~12.6k samples. R 4134→180 Ω. Floor agrees with B1 (173 vs 180 Ω) → good repeatability. |
| B3 | `2026-06-19_fsr_andre_2833_creep1kg.csv` | 2026-06-19 | André Thomas Monteiro | 2833 Ω (measured) | creep @ 1 kg | grams | ~11 min constant 1 kg dead load. **Creep −7.4% (292→270 Ω, −0.67 %/min)**, most drift in first ~4 min. Lift-off tail at the end (still stamped 1000) excluded by the per-bin median. |
| B4 | `2026-06-19_fsr2_andre_2833_sweep.csv` | 2026-06-19 | André Thomas Monteiro | 2833 Ω (measured) | sweep (up+down), **2nd FSR** | grams | 10 levels 50→2000 g. R 3034→158 Ω. Floor agrees with FSR1 (158 vs 173/180 Ω). Part-to-part vs FSR1: ~20–40 % at the light end, ~5–10 % at the floor. |
| B5 | `2026-06-19_fsr2_andre_2833_creep1kg.csv` | 2026-06-19 | André Thomas Monteiro | 2833 Ω (measured) | creep @ 1 kg, **2nd FSR** | grams | **Short (~2 min, not 10)** — creep −5.1 % (253→240 Ω). Consistent with FSR1's early drift; don't compare totals to B3 (different durations). |

> **FSR3 (`2026-06-19_fsr3_andre_2833_sweep.csv`) is VOID** — a botched capture:
> only 200/400 g were stamped and the sensor was never actually loaded (raw maxed
> at 494 → 3034 Ω, far from the ~900/~160 Ω the others hit at 2 kg; the puck
> wasn't transferring force to the pad). Not committed; recapture needed for a
> true N=3 part-to-part point.

### Cross-run findings (FSR1 + FSR2)

- **Saturation:** R drops steeply then flattens toward a ~160–180 Ω floor; the
  knee sits **above the 2 kg scale** → confirms FSRs saturate under seated loads.
- **Repeatability (same part):** FSR1 floor 173 vs 180 Ω across two sweeps.
- **Part-to-part (FSR1 vs FSR2):** ~20–40 % at light loads, converging to ~5–10 %
  at the floor → supports **relative, not calibrated**.
- **Hysteresis (up vs down):** large at light touch (FSR2: down-sweep reads up to
  **−76 %** lower R at 50 g), **negligible (±few %) by ~1 kg**.
- **Takeaway:** all the messiness (spread, noise, hysteresis) lives at the **light
  end**; the heavy/operating regime is clean, repeatable, and direction-independent.

Plots: `../analysis/plot_fsr_curve.py --units g` (sweep, `figures/fsr_curve.png`)
and `../analysis/plot_fsr_creep.py --units g` (creep, `figures/fsr_creep.png`).
