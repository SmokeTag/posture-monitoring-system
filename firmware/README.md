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
- Libraries: **Seeed Arduino LSM6DS3** (IMU, I2C 0x6A) · **nrf_to_nrf**
  (TMRh20, ESB). `Adafruit_TinyUSB` ships with the core. **PostureLink** is in
  this repo (`firmware/libraries/`), hence `--libraries` below.

## Build / upload / monitor (arduino-cli, from the repo root)

```bash
arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense --libraries firmware/libraries firmware/chair
arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p /dev/ttyACM0 firmware/chair
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

Same for `firmware/body`. Upload with the plain `/dev/ttyACMx` path: the
`/dev/serial/by-id/...` symlink fails ("Target is not in DFU mode"). The XIAO is native USB CDC, so the baud rate is
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
firmware/logs in ohms. This serial-only change leaves ESB protocol v2 unchanged:
only the chair needs updating if both boards already run v2.

**ESB protocol v2 — flash both boards together.** Both payloads start with a
version byte (`PROTO_VER`) and have a fixed size; a board running the older
format (v1) is rejected in both directions (the chair counts it in `bad=`, the
body in `bad=` and stays in fallback). Besides pitch/roll, each body packet
reports what the body applied (who decides, reference id, alert, motor/test)
and the age of the chair decision it holds, counted in body samples — the
boards' clocks are never compared. The chair CSV puts that next to its own
decision; timeline and columns: `libraries/PostureLink/PostureLink.h` and the
header of `chair/chair.ino`. Ids are best effort (1-byte body boot id and
chair reference id: 1/255 collisions after a reboot, see `SeqTracker` and
`BodyLink`).
