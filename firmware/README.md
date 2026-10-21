# Firmware

| sketch | board | role |
|--------|-------|------|
| `chair/` | XIAO nRF52840 Sense | chair unit ("brain"): 8 FSRs via CD74HC4067 + ESB receiver; calibrates, decides, logs |
| `body/`  | XIAO nRF52840 Sense | body unit: IMU + ESB transmitter + vibration motor (link-loss fallback) |
| `libraries/PostureLink/` | — | shared ESB protocol + alert rule (both sketches) |
| `archive/xiao_imu_test/` | XIAO nRF52840 Sense | IMU bring-up + posture CSV logging (produced the IMU captures in `data/`) |
| `archive/uno_fsr_test/`  | Arduino Uno | single-FSR402 characterization (produced the Campaign B captures in `data/`) |

`archive/` holds bench sketches that are superseded but kept because they
generated the data in `data/` (reproducibility).

## Toolchain

- Core **Seeeduino:nrf52** (Boards Manager: "Seeed nRF52 Boards", 1.1.13);
  Uno sketch: **arduino:avr**.
- Libraries: **nrf_to_nrf**
  (TMRh20, ESB). `Adafruit_TinyUSB` ships with the core. **PostureLink** is in
  this repo (`firmware/libraries/`), hence `--libraries` below.

## Build / upload / monitor (arduino-cli, from the repo root)

Pick the board by its USB serial number, never by `ttyACM0`/`1` (the numbers
swap between plugs). **Chair firmware on the body runs the motor at full
power** (its mux S0 is the body's motor pin D10).

| Board | USB serial |
|---|---|
| chair | `2C7FE46C8E63BDA0` |
| body  | `3B4A158C694F1777` |

```bash
CHAIR=$(readlink -f /dev/serial/by-id/*2C7FE46C8E63BDA0*)
BODY=$(readlink -f /dev/serial/by-id/*3B4A158C694F1777*)
arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense --libraries firmware/libraries firmware/chair
arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p "$CHAIR" firmware/chair
arduino-cli monitor -p "$CHAIR" -c baudrate=115200
```

Same for `firmware/body` with `"$BODY"`. `readlink -f` matters: uploading
through the `/dev/serial/by-id/...` symlink itself fails ("Target is not in
DFU mode"). That message also shows up now and then on a plain port: just run
the upload again. The XIAO is native USB CDC, so the baud rate is
ignored; serial input is line-buffered (type, then Enter).

## Tests (host, no board)

```bash
make -C firmware/test
```

Unit tests for `libraries/PostureLink` (link freshness, seq bookkeeping,
decision age, payload validation, non-blocking serial output), plus
`chair.ino` and `body.ino` themselves built on host stubs (`test/stub/`: fake
radio, USB serial, IMU) and driven through link loss, duplicates, reboots, a
stalled USB host and a `millis()` wrap; `test_system` runs both sketches
together over a fake radio (decision → applied → reported timeline, fallback,
outage). `analysis/test_live_posture.py` parses that run's chair output.
`test_fastmath` also runs with the board core's `-Ofast` setting: invalid-IMU
heartbeats must accept NaN angles, healthy packets must reject them, and CSV
must preserve `nan`, infinities and negative zero. Protocol classification and
formatting inspect IEEE-754 bits, because `-Ofast` can remove floating-point
`isnan()`/`v != v` checks. No global compiler or Arduino library flags change.

## Serial output

Both sketches write through `libraries/PostureLink/SerialOut.h`: a line goes
out whole only if the USB FIFO has room, so a host that holds the port open but
stops reading can't stall `loop()` (and, on the chair, the ESB ACKs). Dropped
lines are counted in the stats line (`sdrop`/`edrop`); the header is re-sent
whenever a host opens the port.

The chair's `c8..c15` columns always contain raw 12-bit ADC counts (0..4095,
open FSR ≈0), announced by `# unit=raw`. Headers are sent automatically on
connection/reconnection; there is no header command. The live view opens the
port with DTR low, then raises it after pyserial's initial input flush so the
header is not discarded. `analysis/live_posture.py` converts these
counts to conductance/resistance with the 3.3 kΩ divider and still reads older
firmware/logs in ohms.

**ESB protocol v3 — flash both boards together.** Both payloads start with a
version byte (`PROTO_VER`) and have a fixed size; a board running the older
formats (v1/v2) are rejected in both directions (`ver=` counts incompatible
formats, `bad=` counts malformed payloads). Only valid v3 ACKs keep control
fresh; the body falls back after 1 s without one. Each USB header includes
`# proto=3`. Besides pitch/roll, each body packet
reports what the body applied (who decides, reference id, alert, motor/test)
and the age of the chair decision it holds, counted in body samples — the
boards' clocks are never compared. The chair CSV puts that next to its own
decision; timeline and columns: `libraries/PostureLink/PostureLink.h` and the
header of `chair/chair.ino`. Ids are best effort (1-byte body boot id and
chair reference id: 1/255 collisions after a reboot, see `SeqTracker` and
`BodyLink`).

## IMU validity (protocol v3)

The active body sketch uses the local checked `Lsm6Accel.h` reader on Wire1,
not the Seeed float-read helpers (which hide I2C errors/short reads). It checks
WHO_AM_I, writes and reads back the accelerometer configuration (416 Hz,
±16 g, BDU + auto-increment), powers down the unused gyro, checks XLDA for new
data, then reads all six accelerometer bytes in one burst. The global Arduino
libraries/core are untouched. Register reference: [ST AN5130](https://www.st.com/resource/en/application_note/an5130-lsm6ds3trc-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf).

`imu` is the validity of the **current** sample, independent of the radio:

| code | meaning |
|---|---|
| 0 | complete, fresh, usable gravity vector |
| 1 | not initialized / reinitialization pending |
| 2 | I2C failure or incomplete read |
| 3 | XLDA reports no fresh data |
| 4 | saturated axis or gravity magnitude <0.25 g (also possible during motion/free fall) |

An invalid sample has `nan` pitch/roll/deviation. It is a valid radio heartbeat
and does not count as `bad=` or renew a posture decision's basis. The chair
keeps the link up, turns its alert off, resets the hold and aborts any partial
calibration; a valid saved reference stays. Calibration requests while the
latest sample is invalid are ignored. The body suppresses posture alerts in
both chair control and fallback. Recovery restarts the fallback/chair hold;
a chair alert based on a sample before the last failure is also inhibited.

The body CSV appends `imu,inh`. The chair CSV puts `imu,binh` immediately before
`c8..c15` (`imu=-1` while body silent). `inh`/BF_INH reports that a chair alert
was suppressed locally; `binh` is that applied report in the next packet, just
like `balert`. The panel shows the guard explicitly rather than a false apply
mismatch. Motor tests remain explicit (`test`/`btest`), can run with invalid
IMU data, and are not posture alerts. Historical v1/v2 logs show IMU health as
unknown. The panel hides the posture point/trail while the latest sample is
invalid; FSR data remains available.

After 20 consecutive failed samples, the body attempts checked reconfiguration
at most once per second, while continuing its 20 Hz heartbeat. Startup failure
also retries. `BodyPacket` is 19 bytes; `ChairAck` remains 15 bytes but requires
version 3. `ack = ctl + bad + ver`. USB lines use a fixed 176-byte content
buffer (still within TinyUSB's 256-byte FIFO): widest chair row 129 bytes,
body row 63; maximum synthetic stats plus drop counters fit (159/161 bytes).

On Seeeduino:nrf52 1.1.13 the body enables the core's DWT cycle clock before
starting the radio. Without it, `micros()` uses 1024-Hz RTOS ticks (~977 µs),
coarser than the driver's ~800 µs ACK timeout. Changing the IMU read duration
exposed a TX phase where that clock expired before ACKs arrived. This changes
the timeout clock, not the protocol, retries or sampling/alert timing (which
use `millis()`). The core divides the 32-bit cycle counter by 64: its `micros()`
resets about every 67 s, rather than wrapping at UINT32_MAX. A radio wait that
crosses that reset can end early; the next retry starts on the new counter.
This clock is not used for link freshness, calibration, holds or motor pulses.

Limits: the installed Wire implementation has unbounded waits if a bus is
wedged without an error event. PWM is switched off before every I2C operation
and restored after it returns, so that failure leaves the motor off, but the
body cannot keep sending or recover in software while Wire is stuck. The
brief PWM interruption is **not measured on hardware**. XLDA is not proof of
correct contents: plausible corrupt data or a sensor frozen while still
asserting XLDA is not detected. Host stubs cover errors that return, not a
wedged physical bus. The chair's 32-bit millis wrap is still not faithfully
reproduced by host unsigned-long arithmetic; this pre-existing test limitation
is outside this change.

### Bench validation pending

1. Flash **both** boards, verify `Device programmed` plus serial (DFU failure
   may still exit 0). Reopen ports without commands: one v3 header per connect.
2. Baseline: `imu=0`, `inh/binh=0`, `ver=bad=0`, 20 Hz and decision/applied joins
   consistent. Compare retx with a baseline in the same RF conditions.
3. Button/serial calibration, tilt/return, alert pulses and a manual motor test:
   confirm vibration and the new read guard's PWM timing electrically if needed.
4. Chair power cycle/body reset and fallback with the saved reference; recovery
   must not buzz from a previous session's alert. Check all headers on reconnect.
5. Invalid-I2C/short-read/stale/config-loss scenarios are host-injected only;
   do not unplug/solder the onboard sensor as a bench test. Pressure and posture
   accuracy are separate validations.
