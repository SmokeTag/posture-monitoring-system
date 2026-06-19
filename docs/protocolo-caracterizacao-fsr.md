# Procedure — FSR402 resistance-vs-load characterization

> Standalone SOP for **Campaign B** of the midterm plan
> ([`plano-medicoes-midterm.md`](plano-medicoes-midterm.md)). Goal: map FSR
> resistance against applied force, find the **saturation knee**, and confirm the
> "treat FSRs as *relative* pressure/contact, not calibrated force" framing.
> ~15–20 min.

## What it produces

A `data/*fsr*.csv` of paired **(applied load, FSR resistance)** readings across a
force sweep, → an R-vs-load curve with the saturation point marked
(`analysis/figures/fsr_curve.png`).

## You need

- **Arduino Uno** (+ USB cable).
- **1 × FSR402**, a **fixed resistor** for the divider (start **3.3 kΩ**, the
  firmware default — note whichever you use), breadboard + jumpers.
- A **bathroom scale**.
- A **rigid flat puck** — a coin, bottle cap, or small hard disc ~the size of the
  FSR's 12.7 mm pad — to press force evenly onto the sensing area.
- A **hard flat surface** to put the FSR on (a soft surface spreads the load and
  ruins the reading).

## Wiring (voltage divider)

```
   5V ----[ FSR402 ]----+----[ 3.3 kΩ ]---- GND
                        |
                       A0   (analog input)
```

As force rises, FSR resistance drops and the A0 voltage rises. The sketch turns
that back into resistance (ohms) — the board-independent quantity.

## Step 1 — flash the firmware

```bash
arduino-cli compile --upload -p /dev/ttyACM0 --fqbn arduino:avr:uno firmware/uno_fsr_test
```
(Uno clone with a CH340 chip: it shows up as `/dev/ttyUSB0`; genuine boards as
`/dev/ttyACM0`.)

## Step 2 — start logging to file

Pick a name containing `fsr` (the plot script auto-finds the newest such file):
```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee "data/2026-06-19_fsr_andre_3k3.csv"
```
Type `l` + Enter → CSV mode on (header `t_ms,load_kg,raw,ohms` appears).

## Step 3 — sweep the load (the core measurement)

Stack: **hard surface → FSR (flat) → puck on the FSR pad → bathroom scale on
top**, so pressing the scale down drives a known force through the puck onto the
FSR. (Or invert: FSR on the scale, press down through the puck — whichever lets
you read the scale while pressing.)

For each force level, light touch → up toward body weight:

1. Press until the scale reads a target (e.g. 1, 2, 5, 10, 20, 35, 50, 70 kg).
2. **Type that scale reading in kg** + Enter (e.g. `10.0`) — it stamps `load_kg`.
3. Hold steady ~3–5 s so several samples log at that force.
4. Move to the next force. Aim for **~8–10 levels**; do **2 full sweeps** if time.

> Press through the puck, not your fingertip — you want force concentrated on the
> 12.7 mm pad. Keep the FSR flat; don't bend it.

Type `0` + Enter (or just lift off) for the no-load baseline. `ohms = -1` in the
log means open / no force — expected when unloaded.

## Step 4 — stop and plot

`Ctrl-C`, then:
```bash
.venv/bin/python analysis/plot_fsr_curve.py        # newest data/*fsr*.csv
.venv/bin/python analysis/plot_fsr_curve.py --csv data/<file>.csv --rfixed 3300
```
It writes `analysis/figures/fsr_curve.png` and prints the **saturation knee** and
**floor resistance**.

## Step 5 — read the result & pick the divider

- The **knee** (where R stops dropping and flattens onto a floor) is the
  saturation point. If it sits **well below body weight**, the FSR saturates under
  seated loads → confirms "use as relative pressure / contact, not force."
- If the interesting region (light–moderate contact) is cramped, **re-run with a
  different `R_FIXED`**: lower (~1 kΩ) pushes saturation higher / favors heavy
  loads; higher (~10 kΩ) gives more resolution at light touch. Record the value —
  it scales the resistance you can resolve. Note your chosen divider + why.

## Step 6 — register the run

Add a row to [`../data/README.md`](../data/README.md) (file, date, subject,
`R_FIXED`, notes). The FSR runs share the data folder with the IMU captures.

## Bonus (optional) — 6-channel seat weight map

`firmware/uno_fsr6_test` reads 6 FSRs on A0–A5 (no mux) and logs
`t_ms,tag,r0..r5`. Lay them in a rough seat pattern, capture empty vs seated
(`0`/`1` tags) → a left/right + front/back contrast figure. The other Campaign B
deliverable; do it if the curve is done and you have time.

## Quality checklist

- FSR on a **hard, flat** surface; force through a **rigid puck** on the pad.
- ~8–10 load levels from light touch to ~body weight; a few seconds held each.
- `plot_fsr_curve.py` shows R **dropping then flattening** (a clear knee).
- `R_FIXED` recorded; consider a second sweep with a different divider.
