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
- **Chair PCB is BUILT (Sept 2026):** XIAO + CD74HC4067 + 3.3 kΩ dividers + FSR
  connectors on mux channels **C8–C15 (8 FSRs)**; the FSRs themselves are still
  loose (not yet placed in the chair). 4 spare FSR402s on hand.
  - **Pin map (mux → XIAO):** SIG → **D0 (A0)** · S0 → **D10** · S1 → **D3** ·
    S2 → **D2** · S3 → **D1** · EN → GND · VCC → 3V3 out · GND → GND.
    Calibrate button **D9 → GND** (`INPUT_PULLUP`, software debounce).
  - **Divider per channel:** `GND — 3.3 kΩ — Cn (sense node) — FSR402 — 3V3`.
    Pull-down topology: the ADC voltage **rises** with force
    (V = 3.3 V · R_fixed / (R_fixed + R_FSR)); open FSR reads ≈0 V.
  - **Mux on the SENSE side, never in the divider current path:** the HC4067's
    ~100 Ω R_on at 3.3 V is comparable to the 158–180 Ω loaded-FSR floor.
  - **R_fixed = 3.3 kΩ:** centered on the chair's contact range and within
    FSR402's ~1 mA/cm² current guideline.
- Mounting layout **4 seat + 4 backrest**

### Body unit (sensing the upper body)
- **Seeed Studio XIAO nRF52840 "Sense"** worn on the back, just below the neck.
- Uses the onboard **6-axis IMU (LSM6DS3TR-C)** to detect spine curvature /
  slouching of the upper body (the part not in contact with the chair).
- Powered by a **single-cell (1S) 3.7 V LiPo** (JYH LIP103450, 1800 mAh, cell
  cutoff 2.75 V). Firmware cutoff: on battery, VBAT < 3.30 V (3 readings, motor
  off) → System OFF; boot needs ≥ 3.40 V; never on USB; wakes on reset or USB
  plug-in. 4 s watchdog. VBAT goes to the chair CSV (`vbat`, ESB proto v4).
  **Never drive P0.14 or P0.13 HIGH.** Details: `firmware/README.md`.
- Drives a **vibration motor** that activates when problematic posture is
  detected. Coin motor 2.5–4 V / ≤90 mA (too much for a GPIO): **D10 → 1 kΩ →
  NPN (BC337) base**, emitter → GND, collector → motor − ; motor + → **3V3**;
  flyback diode (likely 1N4148 — marking unconfirmed) across the motor
  (cathode at 3V3) + 33 nF ceramic ("333"). PWM sets strength.
- **Body↔chair link = Nordic ESB** (low latency, built-in auto-ack/retransmit);
  body = transmitter (pitch/roll, 20 Hz), chair = receiver; the decision rides
  back in the **ACK payload**. Protocol + alert rule are shared in
  `firmware/libraries/PostureLink`. BLE only if a phone joins later.
  Library **nrf_to_nrf** (TMRh20; drives the RADIO directly, no SoftDevice).
  It is polled — the receiver's ACK is sent inside `radio.available()`, so the
  chair `loop()` must never block (the TX waits only ~0.8 ms for an ACK
  payload). Bench @ ~1 m: 0 lost (20 Hz), but ~10–35 retx/s — tune in the
  power work.
- **Link loss → body fallback:** after 1 s without a valid ChairAck the body applies the
  same alert rule alone, with the last reference the chair sent.
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
