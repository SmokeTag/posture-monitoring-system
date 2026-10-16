# Firmware

| sketch | board | role |
|--------|-------|------|
| `body/`  | XIAO nRF52840 Sense | body unit ("brain"): IMU + ESB receiver + vibration motor |
| `chair/` | XIAO nRF52840 Sense | chair unit: 8 FSRs via CD74HC4067 + ESB transmitter |
| `archive/xiao_imu_test/` | XIAO nRF52840 Sense | IMU bring-up + posture CSV logging (produced the IMU captures in `data/`) |
| `archive/uno_fsr_test/`  | Arduino Uno | single-FSR402 characterization (produced the Campaign B captures in `data/`) |

`archive/` holds bench sketches that are superseded but kept because they
generated the data in `data/` (reproducibility).

## Toolchain

- Core **Seeeduino:nrf52** (Boards Manager: "Seeed nRF52 Boards", 1.1.13);
  Uno sketch: **arduino:avr**.
- Libraries: **Seeed Arduino LSM6DS3** (IMU, I2C 0x6A) · **nrf_to_nrf**
  (TMRh20, ESB). `Adafruit_TinyUSB` ships with the core.

## Build / upload / monitor (arduino-cli, from the repo root)

```bash
arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense firmware/body
arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p /dev/ttyACM1 firmware/body
arduino-cli monitor -p /dev/ttyACM1 -c baudrate=115200
```

Same for `firmware/chair`. The XIAO is native USB CDC, so the baud rate is
ignored; serial input is line-buffered (type, then Enter).
