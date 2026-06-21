/*
 * Arduino Uno (ATmega328P) — 6-channel FSR402 array (weight-distribution map)
 * ---------------------------------------------------------------------------
 * Purpose: read SIX FSR402 sensors at once on A0..A5 and log each as a
 *          resistance (ohms), to produce a seat weight-distribution figure
 *          (left/right, front/back) for the midterm report — WITHOUT the
 *          CD74HC4067 mux (the Uno has exactly 6 analog inputs).
 *
 * This complements firmware/uno_fsr_test (single FSR, R-vs-load curve). Use
 * that one to characterize/saturate one sensor; use THIS one for the spatial
 * load map. The mux + 10 channels come later on the XIAO.
 *
 * Wiring — one independent voltage divider per channel (6 total):
 *
 *     5V ----[ FSR402 ]----+----[ R_FIXED ]---- GND
 *                          |
 *                         A0   (repeat for A1..A5; each FSR + its own R_FIXED)
 *
 *   As force increases, FSR resistance drops, so the A_n node voltage rises.
 *   All six FSRs share the 5V rail and GND; each needs its OWN fixed resistor.
 *
 * Notes:
 *   - Resistance (ohms) is the portable quantity — independent of board
 *     voltage/ADC — so it transfers to the 3.3 V XIAO later. Record ohms.
 *   - Tune R_FIXED to the sitting-load range: start 10k; lower (~3.3k) if
 *     sensors pin near saturation under body weight. Keep all six equal so
 *     channels are directly comparable.
 *   - A channel with no/very light contact reads as "open" -> logged as -1.
 *
 * --- CSV LOGGING ------------------------------------------------------------
 * Emits one comma-separated row per sample after a single header line:
 *
 *   t_ms,tag,r0,r1,r2,r3,r4,r5      (resistances in ohms; -1 = open/no contact)
 *
 * Diagnostic lines are '#'-prefixed (pandas: read_csv(comment='#')). Send a
 * digit '0'..'9' over serial to stamp the `tag` column — e.g. 0=empty seat,
 * 1=seated upright, 2=leaning left, ... — so empty vs seated trials live in
 * one capture. (Just run the monitor without redirect to eyeball values.)
 *
 * Capture workflow (see docs/protocolo-caracterizacao-fsr.md, "6-channel seat weight map"):
 *   arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200 | tee weightmap.csv
 *   # type a digit + Enter to switch tag between empty / seated / leaning trials
 *
 * Build+upload / monitor (arduino-cli; Uno on /dev/ttyACM0, classic Nano on
 * /dev/ttyUSB0 with fqbn arduino:avr:nano:cpu=atmega328):
 *   arduino-cli compile --upload -p /dev/ttyACM0 \
 *     --fqbn arduino:avr:uno firmware/uno_fsr6_test
 *   arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
 */

const int   FSR_PINS[6] = {A0, A1, A2, A3, A4, A5};
const float V_REF_ADC   = 5.0;       // ADC reference (Uno default = 5V)
const float V_DIVIDER   = 5.0;       // supply feeding the dividers
const float R_FIXED     = 3300.0;    // fixed resistor per channel, ohms
const int   ADC_MAX     = 1023;      // 10-bit ADC on the Uno
const int   SAMPLE_MS   = 100;       // 10 Hz — plenty for a static load map

int tag = 0;   // '0'..'9' stamps the tag column (trial label)

// Convert one ADC reading to FSR resistance via the divider equation.
//   v = V_DIVIDER * R_FIXED / (R_fsr + R_FIXED)
//     => R_fsr = R_FIXED * (V_DIVIDER - v) / v
// Returns -1 for an effectively open sensor (no/very light force).
float fsrResistance(int raw) {
  float v = raw * (V_REF_ADC / ADC_MAX);
  if (v <= 0.05) return -1.0;
  return R_FIXED * (V_DIVIDER - v) / v;
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c >= '0' && c <= '9') {
      tag = c - '0';
      Serial.print("# tag=");
      Serial.println(tag);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("# 6-channel FSR402 array on A0..A5. Resistance in ohms; -1 = open.");
  Serial.println("# Send a digit '0'-'9' to set the tag column.");
  Serial.println("t_ms,tag,r0,r1,r2,r3,r4,r5");
}

void loop() {
  handleSerialCommands();

  Serial.print(millis());
  Serial.print(',');
  Serial.print(tag);
  for (int i = 0; i < 6; i++) {
    float r = fsrResistance(analogRead(FSR_PINS[i]));
    Serial.print(',');
    Serial.print(r, 0);   // integer ohms (-1 when open)
  }
  Serial.println();

  delay(SAMPLE_MS);
}
