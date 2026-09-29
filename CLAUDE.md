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
  - **Mux on the SENSE side, never in the divider current path:** the HC4067's
    ~100 Ω R_on at 3.3 V is comparable to the 158–180 Ω loaded-FSR floor.
  - **R_fixed = 3.3 kΩ:** centered on the chair's contact range and within
    FSR402's ~1 mA/cm² current guideline.
- Mounting layout **4 seat + 4 backrest**

### Body unit (sensing the upper body)
- **Seeed Studio XIAO nRF52840 "Sense"** worn on the back, just below the neck.
- Uses the onboard **6-axis IMU (LSM6DS3TR-C)** to detect spine curvature /
  slouching of the upper body (the part not in contact with the chair).
- Powered by a **single-cell (1S) 3.7 V LiPo**
- Drives a **vibration motor** that activates when problematic posture is
  detected.
- **Chair→brain link = Nordic ESB** (low latency, built-in auto-ack/retransmit);
  chair = transmitter, brain = receiver. BLE only if a phone joins later.
- **Calibration = per-user, by button:** alert on deviation from a captured
  "upright" reference, not on absolute angle. The button lives on the chair unit
  (XIAO has only RESET) and sends a calibrate command over ESB; stubbed for now
  by serial `'c'`. Rationale: `docs/decisao-calibracao-vs-limiar-absoluto.md`.

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

## Where things are
- Current phase + backlog: `TASKS.md`
- Firmware: `firmware/` · Analysis (venv `.venv`): `analysis/README.md`
- Results: `analysis/VALIDATION.md` (IMU), `analysis/FSR_FINDINGS.md` + `data/README.md` (FSR)
- Protocols/decisions: `docs/`