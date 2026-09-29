/*
 * XIAO nRF52840 Sense (chair unit) — read the 8 FSR402s through the mux
 * ---------------------------------------------------------------------
 * Scans CD74HC4067 channels C8..C15 (one FSR each), reads the shared SIG line
 * on A0 and prints one CSV row per scan.
 *
 * Per-channel divider (pull-down; mux sits on the SENSE side only):
 *
 *     3V3 ----[ FSR402 ]----+----[ R_FIXED 3.3k ]---- GND
 *                           |
 *                          Cn --(mux)--> SIG --> A0
 *
 *   V = VDD * R_FIXED / (R_FIXED + R_FSR): rises with force, open FSR ~0 V.
 *
 * The ADC uses reference AR_VDD4 (full scale = VDD), i.e. it is RATIOMETRIC
 * with the divider supply, so R_FSR = R_FIXED * (ADC_MAX - raw) / raw holds
 * without knowing the exact 3V3 rail voltage.
 *
 * Mux pins: S0=D10  S1=D3  S2=D2  S3=D1  (EN tied to GND, always on).
 * Channels 8..15 => S3 is always HIGH; S2..S0 = the FSR index 0..7.
 *
 * Output (prose lines are '#'-prefixed; pandas read_csv(comment='#')):
 *   t_ms,c8,c9,...,c15
 * Default unit is ohms (-1 = open / no force). Send 'r' to toggle raw ADC
 * counts (0..4095) — handy for the bring-up check "open FSR ~ 0".
 *
 * Build / upload / monitor (arduino-cli, run from the repo root):
 *   arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense firmware/chair
 *   arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p /dev/ttyACM0 firmware/chair
 *   arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
 */

#include <Adafruit_TinyUSB.h>                  // needed for USB Serial on this core

const int   SIG_PIN    = A0;
const int   SEL_PINS[4] = {D10, D3, D2, D1};   // S0, S1, S2, S3
const int   FIRST_CH    = 8;                   // FSRs on C8..C15
const int   N_FSR       = 8;

const float R_FIXED     = 3300.0;              // ohms (nominal; use measured if known)
const int   ADC_MAX     = 4095;                // 12-bit
const int   OPEN_RAW    = 8;                   // below this => open / no force (~6 mV)
const int   SETTLE_US   = 50;                  // mux switch + SIG node settling
const int   SAMPLE_MS   = 50;                  // 20 Hz scan rate

bool rawMode = false;                          // 'r' toggles ohms <-> raw counts

void selectChannel(int ch) {
  for (int b = 0; b < 4; b++) digitalWrite(SEL_PINS[b], (ch >> b) & 1);
}

int readChannel(int ch) {
  selectChannel(ch);
  delayMicroseconds(SETTLE_US);
  analogRead(SIG_PIN);                         // discard: first conversion after switching
  int raw = analogRead(SIG_PIN);
  return raw < 0 ? 0 : raw;
}

float rawToOhms(int raw) {
  // raw/ADC_MAX = R_FIXED / (R_FIXED + R_FSR)  =>  R_FSR = R_FIXED * (ADC_MAX - raw) / raw
  return (raw >= OPEN_RAW) ? R_FIXED * (ADC_MAX - raw) / raw : -1.0;
}

void printHeader() {
  Serial.print(rawMode ? "# unit=raw\n" : "# unit=ohms (-1 = open)\n");
  Serial.print("t_ms");
  for (int i = 0; i < N_FSR; i++) { Serial.print(",c"); Serial.print(FIRST_CH + i); }
  Serial.println();
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      rawMode = !rawMode;
      printHeader();
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (int b = 0; b < 4; b++) pinMode(SEL_PINS[b], OUTPUT);
  analogReference(AR_VDD4);                    // full scale = VDD -> ratiometric
  analogReadResolution(12);
  analogSampleTime(10);                        // us; source impedance <= R_FIXED

  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // give the USB monitor a moment
  Serial.println("# chair FSR scan: C8..C15. 'r' = toggle ohms/raw.");
  printHeader();
}

void loop() {
  pollSerial();

  unsigned long t = millis();
  Serial.print(t);
  for (int i = 0; i < N_FSR; i++) {
    int raw = readChannel(FIRST_CH + i);
    Serial.print(',');
    if (rawMode) Serial.print(raw);
    else         Serial.print(rawToOhms(raw), 0);
  }
  Serial.println();

  long wait = SAMPLE_MS - (long)(millis() - t);
  if (wait > 0) delay(wait);
}
