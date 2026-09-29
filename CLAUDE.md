# Posture Monitoring System (TCC)

## Project Overview
A wearable + instrumented-chair system that detects poor sitting posture in
people who sit in office chairs for long periods and gives them real-time
feedback (a vibration alert) so they correct themselves.

The system fuses two sources of information:
1. **How body weight is distributed on the chair** (pressure sensors).
2. **The orientation/curvature of the upper back** (worn sensor IMU).

## Git / commits (working agreement)
**Claude owns the commits for this repo.** 
Claude stages and writes the commit itself — clear message, logical grouping, 
no further prompting. Defaults:
- Commit straight to `main` unless told otherwise.
- Group by intent (tooling vs data vs docs) rather than one giant commit.
- Never commit data that's botched/void

## Hardware

### Chair unit (sensing the contact surface)
- **8 × FSR402 force-sensitive resistors** placed in the office chair (seat +
  backrest) to monitor weight distribution and whether the person is seated
  correctly. FSRs are analog (resistance drops as applied force increases).
- A **second XIAO nRF52840 Sense** reads the FSRs and transmits the
  data wirelessly to the body unit over Nordic ESB.
- **Chair PCB is BUILT (Sept 2026):** XIAO + CD74HC4067 + 3.3 kΩ dividers + FSR
  connectors on mux channels **C8–C15 (8 FSRs)**; the FSRs themselves are still
  loose (not yet placed in the chair). 4 spare FSR402s on hand.
  - **Pin map (mux → XIAO):** SIG → **D0 (A0)** · S0 → **D10** · S1 → **D3** ·
    S2 → **D2** · S3 → **D1** · EN → GND · VCC → 3V3 out · GND → GND.
  - **Divider per channel:** `GND — 3.3 kΩ — Cn (sense node) — FSR402 — 3V3`.
    Pull-down topology: the ADC voltage **rises** with force
    (V = 3.3 V · R_fixed / (R_fixed + R_FSR)); open FSR reads ≈0 V.
- Mounting layout **4 seat + 4 backrest**

### Body unit (sensing the upper body)
- **Seeed Studio XIAO nRF52840 "Sense"** worn on the back, just below the neck.
- Uses the onboard **6-axis IMU (LSM6DS3TR-C)** to detect spine curvature /
  slouching of the upper body (the part not in contact with the chair).
- Powered by a **single-cell (1S) 3.7 V LiPo**
- Drives a **vibration motor** that activates when problematic posture is
  detected.
- The nRF52840 supports **BLE**, so it is the natural wireless endpoint.

## Data Flow
```
[8 pressure sensors] -> [chair MCU] --wireless--> [XIAO nRF52840 (body)]
                                                          |
                                          [IMU / upper-back orientation]
                                                          |
                                              posture classification
                                                          |
                                          bad posture? -> [vibration motor]
```
The body XIAO is the "brain": it combines incoming chair data with its own IMU
measurement and decides when to buzz.

## Decisions so far
- **Chair sensors = FSR402 array — 8 in the initial prototype** — read
  via a CD74HC4067 multiplexer (16:1, so both counts fit with no redesign) —
  **mux on the SENSE side** (one divider per channel; the mux only selects which
  node reaches the ADC), never in the divider current path: the HC4067's ~100 Ω
  R_on at 3.3 V is comparable to the measured 158–180 Ω loaded-FSR floor.
  **R_fixed = 3.3 kΩ per channel** (centered on the chair's contact range and
  within FSR402's ~1 mA/cm² current guideline). Bench characterization used a
  measured 2833 Ω divider (runs B1–B7).
- **Body XIAO is the brain** — fuses chair data + IMU, decides when to buzz.
- **Chair MCU = second XIAO nRF52840 Sense — CONFIRMED**, so the chair→brain
  link uses **Nordic Enhanced ShockBurst (ESB)**: low-latency, low-overhead,
  built-in auto-ack/retransmit, single chip family. Chair = transmitter,
  brain = receiver. (BLE is the fallback if a phone is later added to the mix.)
- **Calibration = per-user, by button (DECIDED)** — alert on deviation from a
  captured "upright" reference, not on absolute angle. Rationale + validation:
  `docs/decisao-calibracao-vs-limiar-absoluto.md`.

## Firmware
- `firmware/xiao_imu_test/` — XIAO nRF52840 Sense IMU bring-up (reads onboard
  LSM6DS3 accel + gyro over Serial). Needs "Seeed nRF52 Boards" + "Seeed Arduino
  LSM6DS3" library; board = "Seeed XIAO nRF52840 Sense".
  - **Verified working.** Accel magnitude ≈1g at rest. Gyro has a small constant
    zero-rate bias (gY ≈ -3°/s) — calibrate it out if integrating gyro to angle.
  - Toolchain (Arch): needs `adafruit-nrfutil` (AUR `python-adafruit-nrfutil`)
    to compile/upload; user must be in the `uucp` group for `/dev/ttyACM0`.
  - Computes pitch/roll from accel + per-user upright calibration (serial `'c'`).
    **CSV logging mode** (`'l'` toggle) streams
    `t_ms,tag,cal,pitch,roll,dpitch,droll,ax,ay,az,gx,gy,gz`; digits `'0'`–`'9'`
    stamp the `tag` column to label postures/trials in one capture. Prose lines
    are `#`-prefixed so the stream parses directly (`read_csv(comment='#')`).
- `firmware/uno_fsr_test/` — Arduino Uno single-FSR402 test. Reads a voltage
  divider (R_FIXED 3.3k) and prints raw ADC + **computed FSR resistance (ohms)**.
  **CSV mode** (`'l'` toggle) logs `t_ms,load_kg,raw,ohms`; type the bathroom-scale
  reading (kg) to stamp `load_kg` → the R-vs-load curve. Plot + saturation knee:
  `analysis/plot_fsr_curve.py`. Procedure: `docs/protocolo-caracterizacao-fsr.md`.
- `firmware/uno_fsr6_test/` — Arduino Uno 6-channel FSR402 array on A0–A5 (no
  mux; Uno has exactly 6 analog inputs). Logs CSV `t_ms,tag,r0..r5` (ohms, -1 =
  open) for the seat weight-distribution map. Each channel needs its own FSR +
  fixed resistor; `'0'`–`'9'` tags empty/seated/leaning trials.

## Testing / bring-up plan
1. ✅ **IMU bring-up** on the XIAO Sense (board works + IMU reads).
2. ✅ **FSR402 characterization** on the Arduino Uno (Campaign B): R-vs-load,
   saturation floor, creep, hysteresis, part-to-part (N=3). R_fixed chosen = 3.3 kΩ.
   - **Portability rule:** characterize FSRs in **ohms (resistance), not raw ADC
     counts** — resistance is independent of board voltage/ADC, so Uno data
     transfers directly to the 3.3 V XIAO. (Uno = 5 V / 10-bit; XIAO = 3.3 V,
     up to 12-bit. XIAO pins are NOT 5 V tolerant.)
   - Defer the 10-channel mux and the ESB radio to the XIAO; don't build them on
     the Uno.
3. **Chair-unit bring-up** on the built PCB: read C8–C15 through the mux with
   one FSR first (loaded floor should stay ~160–180 Ω — the test that would
   expose a mux-in-the-current-path error), then place all 8 FSRs in the chair
   and capture the seated weight map.
4. **ESB link** chair→brain (sensor frames + calibrate command), then **fused
   alert logic** + vibration motor driver on the body unit.

## Status
IMU bring-up DONE. **Campaign A (IMU posture) CAPTURED + VALIDATED.**
**Campaign B (FSR402 characterization) CAPTURED + VALIDATED** (7 runs, 3 parts).
Midterm partial report delivered (`report/relatorio_parcial.tex`, June 2026).
**Current phase: chair-unit bring-up** — the chair PCB (second XIAO Sense +
CD74HC4067 + 8 dividers) is built; next is mux read-out firmware, placing the
FSRs in the chair, then the ESB chair→brain link and the fused alert logic
(bring-up steps 3–4 above).

**Campaign A results (run 1, André, no neck mount, own chair):** 6 postures × 5
rounds + 7 re-don calibrations in
`data/2026-06-19_andre_no-neck-mount_own-chair.csv`. Validated 15 PASS / 3 WARN /
0 FAIL (`analysis/VALIDATION.md`). Postures separate in pitch/roll (OE2/OE4);
re-don drifts upright 8.0° pitch / 21.6° roll (OE1, n=4 genuine); proposed alert
angle ≈10°. Caveats: **slouch(1)↔head-down(4) need the chair FSRs to separate**
(and this run's postures were pronounced → optimistic margin); tag `5`=laid-back.
Repeat with neck mount + other subjects: `docs/protocolo-captura-imu.md`.

**Campaign B results (2026-06-19, André, 3 × FSR402, Uno, R_fixed 2833 Ω measured,
precision balance ≤2 kg):** runs B1–B7 in `data/2026-06-19_fsr*_andre_2833_*.csv`;
registry + cross-run notes in `data/README.md`, review in `analysis/FSR_FINDINGS.md`.
Shared ~158–180 Ω floor by ~1.6–2 kg (scale-capped upper bound); part-to-part
≤~46 % but seating swing 3–11× at light load → use as relative pressure/contact;
G = 1/R ~linear in force; creep −7…−12 % over ~12 min @ 1 kg; hysteresis ≈6 % in
the seated regime. **Gate every new capture with `validate_fsr.py` first.**
Procedures: `docs/protocolo-caracterizacao-fsr.md`,
`docs/protocolo-fsr-balanca-precisao.md`.

**Analysis tooling:** `analysis/` (Python venv at `.venv`) — IMU:
`validate_postures.py`, `plot_postures.py`, shared `postures.py`; FSR:
`validate_fsr.py`, `plot_fsr_{curve,parttopart,hysteresis,model,creep}.py`,
shared `fsr_lib.py`. Reviews: `analysis/VALIDATION.md` (A),
`analysis/FSR_FINDINGS.md` (B), `analysis/PROJECT_REVIEW.md`. See `analysis/README.md`.
