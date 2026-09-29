# Procedure — FSR402 R-vs-load on a ≤2 kg precision scale (light-contact)

> Bench SOP using a **precision balance that maxes at ~2 kg**. This maps the **light-contact / low-force end** of the FSR402 curve at high
> resolution. It does **NOT** reach the saturation knee (which sits well above
> 2 kg under a seated adult) — that needs a bigger scale in a later run.
> Designed to be followed step-by-step with check-ins; ~30 min incl. creep test.

## Decisions (already made — don't re-derive)

- **`R_FIXED` = 2.833 kΩ** (MEASURED with a multimeter — a nominally-3.3k
  resistor that actually reads 2833 Ω; the firmware uses the measured value, not
  the nominal). Confirmed sensible: this FSR reads **~224 Ω at 1.8 kg**
  (raw ≈ 948), so its range is roughly 220 Ω → tens of kΩ. The divider's
  best-resolution point is the geometric mean of that range (≈ 3 kΩ) → ~2.8k sits
  right on it. 10k would rail the heavy end (~224 Ω → raw ≈ 997 of 1023, no
  resolution above ~1 kg). The **resistance in ohms is divider-independent**, so
  this curve still transfers to the 3.3 V XIAO and to later runs. *(The future
  seated/heavy run, where R goes below ~100 Ω, will want a lower R_FIXED ~1k —
  different run, different scale.)*
- **Type loads in GRAMS** (`200`, `400`, …) — easier than decimals. The CSV
  column is named `load_kg` but for this run it holds **grams**; analysis divides
  by 1000. Note it in `data/README.md`.
- **No direction tags.** The firmware drops any `+`/`-` suffix anyway. To capture
  loading vs unloading **hysteresis**, just sweep **up then back down in one
  file** — the time order recovers direction in analysis.
- **Loads (~8 levels, roughly doubling):** 50, 100, 200, 400, 800, 1200, 1600,
  2000 g. Use whatever objects/press gets you near these; record what the scale
  actually reads.

## Setup (one config for the whole session)

```
   FSR flat on the scale pan  →  rigid puck centered on the 12.7 mm pad
```
- Put a **thin rigid plate/coaster under the FSR** if the pan is glass/delicate.
- You press on the puck for the sweep; you rest a dead weight on it for creep.
- **Never exceed 2 kg** on the scale.
- Press **through the puck**, not a fingertip — concentrate force on the pad.
  Keep the FSR flat; don't bend it.

Wiring (unchanged): `5V —[FSR402]—+—[2.833 kΩ]— GND`, node `+` → `A0`.

## Step 1 — flash + sanity check  → CHECK-IN A

```bash
arduino-cli compile --upload -p /dev/ttyACM0 --fqbn arduino:avr:uno firmware/uno_fsr_test
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```
Watch the plain `raw  V  ohms` view. **Send the `raw / V / ohms` for (a) no load
and (b) a firm press** to confirm wiring + ADC spread before investing time.
Sanity refs: no load → `open/no-force`; ~1.8 kg → raw ≈ 948, ~224 Ω.

## Step 2 — start logging the sweep

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee "data/2026-06-19_fsr_andre_2833_sweep.csv"
```
Press `l`+Enter → CSV mode on (header `t_ms,load_kg,raw,ohms` appears).

## Step 3 — sweep UP then DOWN (one file)  → CHECK-IN B

1. Press to ~50 g, **type `50`+Enter**, hold steady ~5 s (logs ~50 samples).
2. Step up: `100`, `200`, `400`, `800`, `1200`, `1600`, `2000`.
3. Come back **down** through the same loads: `1600`, `1200`, … `50`.
4. Fully release (lift off). `ohms = -1` = open/no force, expected unloaded.

`Ctrl-C` and report → curve gets plotted and checked before more runs.

## Step 4 — creep test (constant load, 10 min)  → CHECK-IN C

FSRs drift under constant load (creep) — this quantifies it. Use **dead weight**,
not a hand-press (you can't hold a press for 10 min).

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee "data/2026-06-19_fsr_andre_2833_creep1kg.csv"
```
1. `l`+Enter for CSV mode.
2. Rest a **pre-weighed ~1 kg dead weight** on the puck; **type `1000`**.
3. **Leave untouched 10 min.** The scale should hold ~1 kg while resistance drifts.
4. Lift off; let it sit ~1 min unloaded to capture recovery, then `Ctrl-C`.

Plotted as resistance-vs-time.

## Step 5 — redo

Repeat the up/down sweep (Step 3) 1–2 more times into new files
(`..._sweep2.csv`, …) for repeatability / hysteresis spread.

## Step 6 — optional, valuable for later

- **Different FSRs:** repeat one sweep with a 2nd/3rd FSR → part-to-part spread
  (±15–25 % typical) → supports "treat as relative, not calibrated."
- **Lower R_FIXED preview:** one sweep at ~1k (set `R_FIXED` in firmware, note
  it) — cross-checks that computed ohms agrees across dividers, and previews the
  heavy-load divider for the future seated run.

## Step 7 — register the run

Add a row to [`../data/README.md`](../data/README.md): file, date, subject,
`R_FIXED` (2.833k, measured), **units = grams**, sweep vs creep, notes.

## Plot

```bash
.venv/bin/python analysis/plot_fsr_curve.py --csv data/<file>.csv --rfixed 2833
```
(Creep R-vs-time may need a small dedicated plot — handled at check-in C.)
