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

Unit tests for `libraries/PostureLink`: the body's link freshness (only a valid
`ChairAck` keeps the link up), the chair's seq bookkeeping (lost/dup/reboot/
wrap) and the non-blocking serial output.

## Serial output

Both sketches write through `libraries/PostureLink/SerialOut.h`: a line goes
out whole only if the USB FIFO has room, so a host that holds the port open but
stops reading can't stall `loop()` (and, on the chair, the ESB ACKs). Dropped
lines are counted in the stats line (`sdrop`/`edrop`); the header is re-sent
whenever a host opens the port. Flash **both** boards together: the body's
`BodyPacket.boot` (reboot detection) is ignored by an older chair, and an older
body (boot = 0) only gets reboots inferred from seq going back.
