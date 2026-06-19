# Procedure — IMU upper-back posture capture (repeatable)

> Standalone SOP for **Campaign A** of the midterm plan
> ([`plano-medicoes-midterm.md`](plano-medicoes-midterm.md)). Open this, follow
> the steps, get one clean `data/*.csv` ready for `analysis/`. ~15 min of capture.
>
> Repeat this run **with the neck mount** and **with other subjects** as those
> become available — each repeat is a new row in the data run registry.

## What it produces

One continuous serial capture of pitch/roll from the worn XIAO while you hold a
fixed sequence of postures, plus a re-don (re-mounting) experiment. Feeds the
posture-separation result (OE2/OE4) and the calibration-drift result (OE1).

## You need

- **XIAO nRF52840 Sense**, strapped on the upper back just below the neck
  (final wearable location). Bare board or neck mount — **record which**.
- USB-C cable to the laptop (everything stays USB-tethered; no battery needed).
- `arduino-cli` set up for the board. Arch notes: `adafruit-nrfutil` installed,
  user in the `uucp` group for `/dev/ttyACM0`.
- A few minutes where you can sit and hold postures.

## Tag map (digits you press during capture)

| tag | posture |
|----:|---------|
| `0` | upright (reference) |
| `1` | forward slouch (lean over the desk) |
| `2` | lean left |
| `3` | lean right |
| `4` | head-down (look down at lap/phone) |
| `5` | reclined / laid-back (slide hips forward, lean torso back on the chair) |
| `9` | **transition / moving** — press during EVERY move; dropped in analysis |

Pitch reads ~45° upright; forward postures read lower, reclining reads higher.

## Step 1 — record the run metadata (before you start)

Write down: **date · subject · mount (none / neck-mount) · chair**. These become
the filename and a row in [`../data/README.md`](../data/README.md):

```
YYYY-MM-DD_<subject>_<mount>_<chair>.csv
e.g. 2026-06-19_andre_no-neck-mount_own-chair.csv
```

## Step 2 — flash the firmware

```bash
arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense firmware/xiao_imu_test
arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p /dev/ttyACM0 firmware/xiao_imu_test
```

## Step 3 — start logging to file

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee "data/<your-filename>.csv"
```

Typed commands send on **Enter** (the monitor is line-buffered). Single chars:
`l` toggle CSV, `c` calibrate (capture current pose as upright), `r` clear
calibration, `0`–`9` set the tag.

1. Sit upright, relaxed, the way you actually sit.
2. Type `l` → CSV mode on (a `# CSV mode ON` line appears; rows start streaming).
3. Type `c` → calibrate. **Settle fully first and wait ~1 s after pressing** —
   don't double-press (run 1 had an immediate re-press that wasted one epoch).

## Step 4 — the posture sequence (≈10 s each, `9` on every move)

Hold each posture still for **~10 s**, press `9` while moving between them:

```
0 (upright, 10s) → 9 → 1 (slouch) → 9 → 2 (lean L) → 9 → 3 (lean R)
                 → 9 → 4 (head-down) → 9 → 5 (reclined) → 9
```

You time it yourself — there is no per-tag timer; the tag just holds until you
change it. Roughly covering each move with `9` is enough.

> **Do one pronounced set AND one subtle set.** Run 1 only did pronounced
> postures, which makes separation look easier than it is (especially
> slouch ↔ head-down). For a realistic margin, repeat the sequence a second time
> performing each bad posture the way you'd *naturally* drift into it.

## Step 5 — re-don / mounting-drift experiment (the OE1 result)

Repeat 4–6 times: **remove the sensor, re-attach it**, sit upright, press `c`
again, then re-run a short posture sequence (`9` between moves). Re-mounting
shifts the captured "upright" by several degrees — that drift is the point.

## Step 6 — stop and validate

`Ctrl-C` to stop the monitor. Then:

```bash
.venv/bin/python analysis/validate_postures.py        # newest data/ file; expect 0 FAIL
.venv/bin/python analysis/plot_postures.py            # writes analysis/figures/*.png
```

If you don't have the env yet: `python3 -m venv .venv &&
.venv/bin/pip install -r analysis/requirements.txt`.

## Step 7 — register the run

Add a row to the table in [`../data/README.md`](../data/README.md) (run #, file,
date, subject, mount, chair, notes — e.g. pronounced vs subtle, anything odd).

## Quality checklist (what "good" looks like)

- `validate_postures.py` → **0 FAIL** (WARNs about the hard 1↔4 pair and the
  `cal`-flag semantics are expected).
- Each posture held ≥ ~10 s; `9` covers every transition.
- Steady `|g|` ≈ 1.0 (the validator checks this — flags a bad sensor read).
- Re-don done ≥ 4× so the drift figure (`04_redon_drift.png`) is meaningful.
