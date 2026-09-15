# Posture Monitoring System (TCC)

## Project Overview
A wearable + instrumented-chair system that detects poor sitting posture in
people who sit in office chairs for long periods and gives them real-time
feedback (a vibration alert) so they correct themselves.

The system fuses two sources of information:
1. **How body weight is distributed on the chair** (pressure sensors).
2. **The orientation/curvature of the upper back** (worn sensor), to catch
   slouching and spine curvature that the chair alone cannot see.

## Git / commits (working agreement)
**Claude owns the commits for this repo.** When André asks to commit (or signals
he doesn't want to think about it), Claude stages and writes the commit itself —
clear message, logical grouping, no further prompting. Defaults:
- Commit straight to `main` unless told otherwise.
- Group by intent (tooling vs data vs docs) rather than one giant commit.
- Never commit data that's botched/void — flag it and leave it out (see the
  since-resolved FSR3 VOID note in `data/README.md` for an example).

## Hardware

### Chair unit (sensing the contact surface)
- **10 × FSR402 force-sensitive resistors** placed in the office chair (seat +
  backrest) to monitor weight distribution and whether the person is seated
  correctly. FSRs are analog (resistance drops as applied force increases).
- A **second XIAO nRF52840 Sense** (confirmed) reads the FSRs and transmits the
  data wirelessly to the body unit over Nordic ESB (see Decisions below).
- FSR characterization was prototyped on an Arduino Uno (Campaign B, done); the
  chair unit proper is built around the XIAO.
- **Chair PCB is BUILT (Sept 2026):** XIAO + CD74HC4067 + 3.3 kΩ dividers + FSR
  connectors on mux channels **C8–C15 (8 FSRs)**; the FSRs themselves are still
  loose (not yet placed in the chair). 4 spare FSR402s on hand.
  - **Pin map (mux → XIAO):** SIG → **D0 (A0)** · S0 → **D10** · S1 → **D3** ·
    S2 → **D2** · S3 → **D1** · EN → GND · VCC → 3V3 out · GND → GND.
  - **Divider per channel:** `GND — 3.3 kΩ — Cn (sense node) — FSR402 — 3V3`.
    Pull-down topology: the ADC voltage **rises** with force
    (V = 3.3 V · R_fixed / (R_fixed + R_FSR)); open FSR reads ≈0 V.
  - Note D1–D3 (A1–A3) are consumed as select lines — fine, only A0 is needed
    because every FSR goes through the mux.
- Mounting layout (where on the seat/backrest each FSR goes) is not yet decided;
  likely **4 seat + 4 backrest** for the 8-FSR prototype.

### Body unit (sensing the unsupported upper body)
- **Seeed Studio XIAO nRF52840 "Sense"** worn on the back, just below the neck.
  *(Confirmed Sense variant — onboard IMU reads live: accel ≈1g at rest.)*
- Uses the onboard **6-axis IMU (LSM6DS3TR-C)** to detect spine curvature /
  slouching of the upper body (the part not in contact with the chair).
- Powered by a **single-cell (1S) 3.7 V LiPo** (the XIAO has an onboard charging
  circuit and battery pads — solder the cell to BAT+/BAT−; no JST onboard).
- Drives a **vibration motor** that activates when problematic posture is
  detected.
- The nRF52840 supports **BLE**, so it is the natural wireless endpoint.

## Data Flow (current understanding)
```
[10 pressure sensors] -> [chair MCU] --wireless--> [XIAO nRF52840 (body)]
                                                          |
                                          [IMU / upper-back orientation]
                                                          |
                                              posture classification
                                                          |
                                          bad posture? -> [vibration motor]
```
The body XIAO is the "brain" (at least to start): it combines incoming chair
data with its own IMU measurement and decides when to buzz. May move the
decision logic elsewhere later if needed.

## Key technical considerations (to keep in mind)
- **FSR402 saturates under body weight — measured.** Each FSR402 is a ~12.7 mm
  sensor; on the bench (Campaign B, rigid puck) all three parts converge to a
  shared **~158–180 Ω floor by ~1.6–2 kg** (an upper bound — the scale capped at
  2 kg). A seated adult loads far past that, so treat readings as **relative
  pressure / weight distribution / contact** (left-vs-right, front-vs-back,
  region loaded?) rather than calibrated force. Conductance G = 1/R is ~linear
  in force and is the better variable for a pressure map. Placement/seating
  matters a lot (seating swing 3–11× ≫ part-to-part ≤~46 %).
- **The FSR array needs an analog multiplexer.** The XIAO nRF52840 breaks out
  **exactly 6 usable analog inputs — A0–A5 = pads D0–D5** (verified in the Seeed
  core's `variant.cpp`); `NUM_ANALOG_INPUTS` says 8 because the nRF52840 SAADC has
  8 channels, but AIN6 (P0.30) is the onboard green LED and AIN7 (P0.31) is the
  battery-sense divider — neither reaches a pad. D6–D10 are digital-only. So even
  the **8-FSR initial prototype** (10 is the final target) needs the mux.
  (Arduino Uno also has 6.) Use a 16-channel mux like the **CD74HC4067** to
  read them all through one ADC pin + 4 select pins. Each FSR needs its **own**
  fixed resistor (voltage divider) — do NOT share one after the mux; see the
  R_on argument under Decisions.
- **Vibration motor cannot be driven directly from a GPIO pin** — it needs a
  transistor/MOSFET driver plus a flyback diode.
- **Battery = single-cell (1S) 3.7 V LiPo — NO "3.3 V" cell or step-down
  needed.** A 1S LiPo swings 3.0–4.2 V; the XIAO's onboard regulator makes the
  3.3 V rail and its charger tops the cell to 4.2 V over USB-C. Do NOT use a
  2S/7.4 V pack. Power-pin map: **5V pin** = USB VBUS (feed 5 V here to run the
  board), **3V3 pin** = regulated 3.3 V *output*, **GPIO/analog = 3.3 V and NOT
  5 V tolerant**.
- **Power budget** matters on the body unit (battery + radio + motor); the chair
  unit's power source (USB/wall vs. battery) is TBD.

## Decisions so far
- **Body unit = XIAO nRF52840 Sense**, using its onboard IMU for upper-back
  orientation (board variant confirmed — IMU streams live accel/gyro).
- **Chair sensors = FSR402 array — 8 in the initial prototype, 10 final** — read
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

## Open questions / decisions to make
- [x] Chair MCU = second XIAO nRF52840 Sense — CONFIRMED.
- [ ] Chair unit power source (USB/wall vs. battery).
- [ ] FSR mounting layout — where on the seat/backrest do the sensors go?
  - **8-FSR prototype, likely:** 4 seat + 4 backrest (PCB has C8–C15 wired).
  - **10-FSR final proposal (not final):** 6 seat (2 cols × 3 rows) + 4 backrest (2 × 2).
- [ ] Is there a companion app / dashboard / data logging, or fully standalone?
- [x] How is "correct" vs. "problematic" posture defined — **DECIDED: per-user
      calibration, alert on deviation** (not fixed absolute thresholds).
  - **Rationale + validation (PT-BR):** `docs/decisao-calibracao-vs-limiar-absoluto.md`
    — absolute thresholds are fragile (mounting offset + per-person upright);
    Campaign A / OE1 measured 8.0° pitch / 21.6° roll re-don drift of the
    "upright" reference, larger than several postures to detect. Still pending:
    physical button, ESB command, flash persistence of the reference.
  - **Calibration trigger = a "calibrate" button on the chair unit.**
    Pressing it (while sitting upright) sends a calibrate command over the
    chair→brain wireless link (ESB); the body XIAO captures the current IMU
    pitch/roll as the per-user "upright" reference and afterwards alerts on
    deviation from it. The button lives on the chair MCU (not the XIAO) because
    the XIAO has no usable user button — only RESET — and the chair MCU already
    has the radio link to the brain. For now this is stubbed by a serial 'c'
    command in `xiao_imu_test` (see Firmware).
- [x] XIAO variant = Sense — CONFIRMED. `arduino-cli board list` auto-detected
      it, and the IMU sketch ran (`imu.begin()` OK + live accel/gyro values).

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
