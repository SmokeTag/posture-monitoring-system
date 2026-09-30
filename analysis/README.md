# Posture data — validation & plotting

Scripts to **validate** the IMU capture (`postures.csv`) and **visualise** it.
The CSV is the serial dump from `firmware/xiao_imu_test/xiao_imu_test.ino`
(XIAO nRF52840 Sense on the upper back). Schema:

```
t_ms,tag,cal,pitch,roll,dpitch,droll,ax,ay,az,gx,gy,gz
```

## Setup (once)

Python 3 with a venv (the repo's `.gitignore` already ignores `.venv/`):

```bash
python3 -m venv .venv
.venv/bin/pip install -r analysis/requirements.txt
```

## Validate the data

```bash
.venv/bin/python analysis/validate_postures.py            # newest capture in data/
.venv/bin/python analysis/validate_postures.py --csv data/<some-run>.csv
```

Prints a PASS/WARN/FAIL report (parse integrity, timing, calibration logic,
sensor health, protocol/posture content) and exits non-zero only on a FAIL.
Re-runnable on any future capture. Full write-up of the current file:
[`VALIDATION.md`](VALIDATION.md).

## Make the figures

```bash
.venv/bin/python analysis/plot_postures.py                # writes analysis/figures/*.png
.venv/bin/python analysis/plot_postures.py --show         # also opens windows
```

| file | what it shows |
|------|---------------|
| `01_timeline.png`      | pitch & roll over the whole capture, shaded per posture, calibration presses marked |
| `02_separation.png`    | pitch-vs-roll clusters — raw (all rounds) **vs** calibrated deviation (the decision space). OE2/OE4 |
| `03_distributions.png` | per-posture pitch & roll box plots |
| `04_redon_drift.png`   | how the "upright" reference drifts across re-don calibrations. OE1 |
| `05_sensor_health.png` | accelerometer `|g|` over time + histogram (steady ≈ 1 g; motion in tag 9) |
| `06_alert_threshold.png` | deviation from upright per posture + a proposed ~10° alert angle |

## Files

- `postures.py` — shared loader + helpers (`load`, `steady`, `segments`,
  `accel_magnitude`, `recompute_angles`, `calibration_epochs`,
  `parse_calibration_comments`), plus the tag map / colours. Import it from your
  own notebooks.
- `validate_postures.py` — the validator (above).
- `plot_postures.py` — the IMU figures (above).
- `live_imu.py` — live plot of the body unit (`firmware/body`) over serial: current
  `(dpitch, droll)` + trail on the recorded clusters, alert circles, alert/LED
  state. `c`/`r` in the window calibrate/clear. `--log` also saves the stream.
### FSR402 tooling (Campaign B)

All share `fsr_lib.py` (robust loader, load-jitter canonicalization, up/down
direction split, power-law fit, run discovery). Inputs are `data/*fsr*.csv` from
`firmware/uno_fsr_test` (press `l`, type each scale reading; precision-balance
runs stamp **grams**). Procedure: `../docs/protocolo-fsr-balanca-precisao.md`.

| script | what it does | figure |
|--------|--------------|--------|
| `validate_fsr.py` | **PASS/WARN/FAIL gate** — auto-detects sweep vs creep; catches a VOID / not-pressed capture and a poorly-seated puck (up/down ratio). **Run first on every new capture.** | — |
| `plot_fsr_curve.py` | single-sweep R-vs-load + saturation knee (now flags "floor = 2 kg-cap reading") | `fsr_curve.png` |
| `plot_fsr_parttopart.py` | N-sensor overlay (FSR1/2/3) + part-to-part spread vs load; seating transients flagged & excluded | `fsr_parttopart.png` |
| `plot_fsr_hysteresis.py` | loading vs unloading loop per load level | `fsr_hysteresis.png` |
| `plot_fsr_model.py` | power-law fit + conductance-is-linear + ADC resolution | `fsr_model.png` |
| `plot_fsr_creep.py` | resistance drift vs time under a constant dead load | `fsr_creep.png` |

```bash
.venv/bin/python analysis/validate_fsr.py --csv data/<run>.csv   # gate a capture
.venv/bin/python analysis/plot_fsr_parttopart.py                 # FSR1/2/3 overlay
.venv/bin/python analysis/plot_fsr_curve.py --units g            # newest sweep
```

### Tag map

`0`=upright, `1`=forward slouch, `2`=lean left, `3`=lean right, `4`=head-down,
`5`=laid-back (hips forward, torso back — highest pitch),
`9`=transition/moving (**dropped** from steady analysis).
