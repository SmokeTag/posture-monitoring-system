# Posture Monitoring System (TCC)

## Project Overview
A wearable + instrumented-chair system that detects poor sitting posture in
people who sit in office chairs for long periods and gives them real-time
feedback (a vibration alert) so they correct themselves.

The system fuses two sources of information:
1. **How body weight is distributed on the chair** (pressure sensors).
2. **The orientation/curvature of the upper back** (worn sensor IMU).

## Scope of changes
**Do not make any change (code, docs, data, config) unless explicitly requested.**
Suggestions and proposed changes are welcome — just ask before applying them.

## Git
**The AI agent owns the commits for this repo.**
It stages and writes the commit itself — clear message, logical grouping,
no further prompting. Defaults:
- **Small / high-confidence changes** → commit straight to `main`.
- **Larger or non-trivial changes** → open a PR (feature branch) so the
  automated review bots can give feedback before merging.
  After publishing, wait for the **2 automated reviews** (~10–15 min), then
  address their feedback until the PR is ready to merge.
- Group by intent (tooling vs data vs docs) rather than one giant commit.
- Never commit data that's botched/void

## Hardware

### Chair unit (sensing the contact surface)
- **8 × FSR402 force-sensitive resistors** placed in the office chair (seat +
  backrest) to monitor weight distribution and whether the person is seated
  correctly. FSRs are analog (resistance drops as applied force increases).
- A **second XIAO nRF52840 Sense** reads the FSRs and is the system's
  **"brain"**: it receives the body unit's IMU over Nordic ESB, decides the
  alert and sends it back. A laptop on its USB sees the whole system in one CSV.
- **Chair electronics FINISHED (Oct 2026):** XIAO + CD74HC4067 + a single
  3.3 kΩ pull-down on SIG + FSR connectors on mux channels **C8–C15 (8 FSRs)**;
  FSRs placed in the chair (Oct 2026). 4 spare FSR402s on hand. Powered by a 1S LiPo with the **same circuit as the body
  unit** — battery not yet tested on the chair.
  - **Pin map (mux → XIAO):** SIG → **D0 (A0)** · S0 → **D10** · S1 → **D3** ·
    S2 → **D2** · S3 → **D1** · EN → GND · VCC → 3V3 out · GND → GND.
    Calibrate button **D9 → GND** (`INPUT_PULLUP`, software debounce).
  - **One shared divider on SIG:** `3V3 — FSR402 — Cn —(mux, R_on)— SIG — 3.3 kΩ — GND`,
    SIG → A0. Pull-down topology: the ADC voltage **rises** with force
    (V = 3.3 V · R_fixed / (R_fixed + R_FSR + R_on)); open FSR reads ≈0 V.
    Unselected FSR nodes float to ~3V3 (no load); no cap on SIG, so the
    firmware's 50 µs settle + discarded read is enough.
  - **Mux R_on (~100 Ω at 3.3 V) is in the divider current path**, in series
    with the FSR. Accepted: its fixed per-channel part is absorbed by the
    per-user calibration, it is far below the FSR402's own spread/creep, and
    the measured floor under body load is ≥ 400 Ω. Subtract ~100 Ω only when
    absolute ohms are needed.
  - **R_fixed = 3.3 kΩ:** ≈ √(R_min · R_max) of the contact range (best
    resolution) and within FSR402's ~1 mA/cm² current guideline.
- Mounting layout **4 seat + 4 backrest**

### Body unit (sensing the upper body)
- **Seeed Studio XIAO nRF52840 "Sense"** worn on the back, just below the neck.
- Uses the onboard **6-axis IMU (LSM6DS3TR-C)** to detect spine curvature /
  slouching of the upper body (the part not in contact with the chair).
- Powered by a **single-cell (1S) 3.7 V LiPo** (JYH LIP103450, 1800 mAh, cell
  cutoff 2.75 V). Firmware cutoff: on battery, VBAT < 3.30 V (3 readings, motor
  off) → System OFF; boot needs ≥ 3.40 V; never on USB; wakes on reset or USB
  plug-in. 4 s watchdog. VBAT goes to the chair CSV (`vbat`, ESB proto v5).
  **Never drive P0.14 or P0.13 HIGH.** Details: `firmware/README.md`.
  Unit FINISHED (Oct 2026), battery connected and working.
- Drives a **vibration motor** that activates when problematic posture is
  detected. Coin motor 2.5–4 V / ≤90 mA (too much for a GPIO): **D10 → 1 kΩ →
  NPN (BC337) base**, emitter → GND, collector → motor − ; motor + → **3V3**;
  flyback diode (likely 1N4148 — marking unconfirmed) across the motor
  (cathode at 3V3) + 33 nF ceramic ("333"). PWM sets strength.
  The buzz shakes the IMU (±10°), so while the PWM runs the body sends the
  last reading taken with the motor off (≤ 1.5 s).
  **Motor mute for data captures:** `live_posture.py --motor-muted` sends
  `'q'` to the chair (`'v'` unmutes) → `CF_MUTE` in the ChairAck; the body
  doesn't buzz on alerts (motor test still does, fallback keeps it), the chair
  still decides/logs `alert`. Never saved: reboot or closing the USB port unmutes.
- **Body↔chair link = Nordic ESB** (low latency, built-in auto-ack/retransmit);
  body = transmitter (pitch/roll, 20 Hz), chair = receiver; the decision rides
  back in the **ACK payload**. Protocol + alert rules are shared in
  `firmware/libraries/PostureLink`.
  The chair also broadcasts a one-way BLE state beacon (~20 Hz); `mostra.py --ble` reads it.
  Without a USB host, an empty seat for 10 s clears calibration.
  Library **nrf_to_nrf** (TMRh20; drives the RADIO directly, no SoftDevice).
  It is polled — the receiver's ACK is sent inside `radio.available()`, so the
  chair `loop()` must never block (the TX waits only ~0.8 ms for an ACK
  payload). Bench @ ~1 m: 0 lost (20 Hz), but ~10–35 retx/s — tune in the
  power work.
- **Chair alert = IMU + FSR fusion** (`PostureLink/Fusion.h`): IMU deviation,
  slid forward (seat rear unloaded), FSR-confirmed lean; reclined, crossed legs and
  empty seat don't alert. Rationale: `docs/decisao-fusao-imu-fsr.md`.
- **Link loss → body fallback:** after 1 s without a valid ChairAck the body applies the
  IMU-only rule (deviation > 10° for 3 s) alone, with the last reference the chair sent.
- **Calibration = per-user, by button:** alert on deviation from a captured
  "upright" reference, not on absolute angle. The button lives on the chair unit
  (XIAO has only RESET), which captures the reference from the body's samples
  (serial `'c'` also works); the chair's LED blinks green while capturing.
  Rationale: `docs/decisao-calibracao-vs-limiar-absoluto.md`.
- **Status LED (both boards, RGB):** orange = no link, blue = linked,
  green = calibrated, red slow = alert, red fast = fault
  (`PostureLink/StatusLed.h`; dimmed on the body for the LiPo); body only:
  3 magenta blinks = battery low, powering off.

## Data Flow
```
[IMU / upper back] -> [body XIAO] --- ESB: pitch/roll ---> [chair XIAO] <- [8 FSRs]
                       |    ^                                      |
                       |    +---- ESB ACK payload: alert ------ posture
                       v                                     classification
               [vibration motor]
```
The chair XIAO is the "brain": it combines its FSRs with the body's IMU and
decides when to buzz; the body drives the motor (and decides alone if the link
drops).

## Repository structure
```
firmware/   Arduino sketches (toolchain/build: firmware/README.md)
analysis/   Python analysis + findings (venv .venv; see README.md)
data/       raw CSV captures (registry: data/README.md)
docs/       protocols, decisions, research (PT-BR)
report/     LaTeX report
TASKS.md    backlog
```
