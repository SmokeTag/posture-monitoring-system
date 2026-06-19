# Posture Monitoring System (TCC)

## Project Overview
A wearable + instrumented-chair system that detects poor sitting posture in
people who sit in office chairs for long periods and gives them real-time
feedback (a vibration alert) so they correct themselves.

The system fuses two sources of information:
1. **How body weight is distributed on the chair** (pressure sensors).
2. **The orientation/curvature of the upper back** (worn sensor), to catch
   slouching and spine curvature that the chair alone cannot see.

## Hardware

### Chair unit (sensing the contact surface)
- **10 × FSR402 force-sensitive resistors** placed in the office chair (seat +
  backrest) to monitor weight distribution and whether the person is seated
  correctly. FSRs are analog (resistance drops as applied force increases).
- A **microcontroller** reads the 10 FSRs and transmits the data wirelessly to
  the body unit. **Leaning toward a second XIAO nRF52840 (plain variant)** so
  the chair↔brain link can use Nordic ESB (see Decisions below).
- Prototyping the FSR reading on an Arduino for now; final radio choice drives
  the board decision.
- Mounting layout (where on the seat/backrest each FSR goes) is not yet decided.

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
- **FSR402 will likely saturate under body weight.** Each FSR402 is a ~12.7 mm
  sensor rated to ~10 kg before its response flattens. Under a seated adult,
  sensors will probably max out, so treat readings as **relative pressure /
  weight distribution / contact** (left-vs-right, front-vs-back, region loaded?)
  rather than calibrated force. Placement matters a lot.
- **10 FSRs need an analog multiplexer.** The XIAO nRF52840 breaks out ~6 analog
  pins (Arduino Uno also ~6). Use a 16-channel mux like the **CD74HC4067** to
  read all 10 through one ADC pin + 4 select pins. Each FSR needs a fixed
  resistor (voltage divider) — can share one divider resistor after the mux.
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
- **Chair sensors = 10 × FSR402**, read via a CD74HC4067 multiplexer.
- **Body XIAO is the brain** — fuses chair data + IMU, decides when to buzz.
- **Chair MCU leaning = second XIAO nRF52840 (plain)**, so the chair→brain link
  can use **Nordic Enhanced ShockBurst (ESB)**: low-latency, low-overhead,
  built-in auto-ack/retransmit, single chip family. Chair = transmitter,
  brain = receiver. (BLE is the fallback if a phone is later added to the mix.)

## Open questions / decisions to make
- [ ] Confirm the chair MCU choice (leaning second XIAO nRF52840) and its power
      source (USB/wall vs. battery).
- [ ] FSR mounting layout — where on the seat/backrest do the 10 sensors go?
- [ ] Is there a companion app / dashboard / data logging, or fully standalone?
- [ ] How is "correct" vs. "problematic" posture defined/calibrated per user
      (fixed thresholds vs. per-person calibration)?
  - **Rationale documented (PT-BR):** `docs/decisao-calibracao-vs-limiar-absoluto.md`
    — why absolute thresholds are fragile (mounting offset + per-person upright)
    and why calibration is favored. Currently leaning calibration button,
    pending experimental validation.
  - **Planned calibration trigger = a "calibrate" button on the chair unit.**
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
  divider and prints raw ADC + **computed FSR resistance (ohms)**.
- `firmware/uno_fsr6_test/` — Arduino Uno 6-channel FSR402 array on A0–A5 (no
  mux; Uno has exactly 6 analog inputs). Logs CSV `t_ms,tag,r0..r5` (ohms, -1 =
  open) for the seat weight-distribution map. Each channel needs its own FSR +
  fixed resistor; `'0'`–`'9'` tags empty/seated/leaning trials.

## Testing / bring-up plan
1. **IMU bring-up** on the XIAO Sense (confirm board works + IMU reads).
2. **FSR402 characterization** on the Arduino Uno: pick the fixed divider
   resistor (start 10k), map resistance-vs-load, find where it saturates under
   realistic seat loads.
   - **Portability rule:** characterize FSRs in **ohms (resistance), not raw ADC
     counts** — resistance is independent of board voltage/ADC, so Uno data
     transfers directly to the 3.3 V XIAO. (Uno = 5 V / 10-bit; XIAO = 3.3 V,
     up to 12-bit. XIAO pins are NOT 5 V tolerant.)
   - Defer the 10-channel mux and the ESB radio to the XIAO; don't build them on
     the Uno.

## Status
IMU bring-up DONE — XIAO Sense confirmed, IMU streaming. Next: FSR402
characterization on the Arduino Uno.

**Active midterm plan (deadline ≈ 2026-06-20):** bench-characterize each sensing
channel independently with on-hand hardware (no 2nd XIAO / mux / battery / chair
yet) for a written partial report. Two campaigns — (A) IMU upper-back posture +
re-don/calibration experiment, (B) FSR402 load characterization + 6-channel
weight map on the Uno (no mux). Full plan, TCC-objective mapping, and tracking
checklist: `docs/plano-medicoes-midterm.md`. Critical path = CSV pitch/roll
logging in `xiao_imu_test` + a 6-channel FSR sketch for the Uno.
