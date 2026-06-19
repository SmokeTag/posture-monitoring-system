/*
 * Arduino Uno / classic Nano (ATmega328P) — FSR402 characterization
 * -----------------------------------------------------------------------
 * Read ONE FSR402 as both raw ADC and FSR resistance (ohms). Resistance is
 * the portable quantity — a curve measured here transfers directly to the
 * XIAO nRF52840 later, even though that board runs at 3.3 V with a different ADC.
 *
 * Wiring (voltage divider):
 *
 *     VCC ----[ FSR402 ]----+----[ R_FIXED ]---- GND
 *                           |
 *                          A0   (analog input)
 *
 *   As force increases, FSR resistance drops, so the A0 node voltage rises.
 *
 * --- R-vs-LOAD CURVE LOGGING (the midterm Campaign B figure) ----------------
 * To map resistance against APPLIED FORCE you must pair each resistance reading
 * with the force you are applying. Press the FSR through a rigid flat puck that
 * sits on a BATHROOM SCALE; the scale tells you the force. Then:
 *
 *   Send 'l' to toggle CSV mode. In CSV mode the sketch emits one row per
 *   sample after a header line:
 *
 *       t_ms,load_kg,raw,ohms
 *
 *   Type a NUMBER (the current scale reading in kg) + Enter to stamp the
 *   `load_kg` column; it holds until you type the next number. So the workflow
 *   is: press to a force, read the scale, type that number, hold steady a few
 *   seconds, then move to the next force. Sweep light touch -> ~body weight.
 *   ohms = -1 means open / no force. Prose lines are '#'-prefixed so the file
 *   parses directly (pandas read_csv(comment='#')).
 *
 * Notes:
 *   - On the Uno or classic Nano (both ATmega328P @ 5V), power the divider
 *     from 5V and keep V_REF_ADC = 5.0. (The Nano's "3V3" pin is just a
 *     low-current tap from the USB chip — the board still runs at 5V.)
 *   - On a XIAO nRF52840, power the divider from 3.3V and set BOTH voltage
 *     constants to 3.3 (and never exceed 3.3 V on an input pin). The printed
 *     resistance is still directly comparable across boards.
 *   - Tune R_FIXED: 3.3k here. Lower it if it saturates too early under sitting
 *     loads; raise it for more sensitivity to light contact. Record which value
 *     you used (it scales the resistance you can resolve).
 *
 * Build+upload / monitor (arduino-cli, Uno on /dev/ttyACM0):
 *   arduino-cli compile --upload -p /dev/ttyACM0 \
 *     --fqbn arduino:avr:uno firmware/uno_fsr_test
 *   arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee data/<file>.csv
 * (classic Nano clone: --fqbn arduino:avr:nano:cpu=atmega328 on /dev/ttyUSB0)
 */

const int   FSR_PIN     = A0;
const float V_REF_ADC   = 5.0;       // ADC reference (Uno default = 5V)
const float V_DIVIDER   = 5.0;       // supply feeding the divider
const float R_FIXED     = 3300.0;    // fixed resistor, ohms
const int   ADC_MAX     = 1023;      // 10-bit ADC on the Uno

const int SAMPLE_MS_LIVE = 200;      // human-readable mode (5 Hz)
const int SAMPLE_MS_CSV  = 100;      // CSV logging mode (10 Hz)

bool   csvMode = false;   // 'l' toggles CSV output for file capture
float  loadKg  = 0.0;     // last scale reading typed (stamps the load_kg column)
String lineBuf = "";      // accumulates a typed line until Enter

float readResistance(int &rawOut) {
  int   raw = analogRead(FSR_PIN);
  rawOut = raw;
  float v = raw * (V_REF_ADC / ADC_MAX);              // node voltage from ADC
  // Divider: v = V_DIVIDER * R_FIXED / (R_fsr + R_FIXED)
  //   => R_fsr = R_FIXED * (V_DIVIDER - v) / v ; v~0 => open / no force.
  return (v > 0.05) ? R_FIXED * (V_DIVIDER - v) / v : -1.0;
}

// Line-based serial: 'l' toggles CSV; any numeric line sets the load stamp.
// (arduino-cli monitor is line-buffered — input arrives on Enter.)
void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  if (s == "l" || s == "L") {
    csvMode = !csvMode;
    if (csvMode) {
      Serial.println("# CSV mode ON. Header follows; type a number (kg) to set load.");
      Serial.println("t_ms,load_kg,raw,ohms");
    } else {
      Serial.println("# CSV mode OFF.");
    }
    return;
  }
  char c0 = s.charAt(0);
  if (c0 == '-' || c0 == '.' || (c0 >= '0' && c0 <= '9')) {
    loadKg = s.toFloat();
    Serial.print("# load_kg=");
    Serial.println(loadKg, 2);
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      handleLine(lineBuf);
      lineBuf = "";
    } else {
      lineBuf += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("# FSR402 characterization. 'l' = CSV mode; type the scale");
  Serial.println("# reading in kg to stamp load. Default view: raw  V_node  R(ohm).");
}

void loop() {
  pollSerial();

  int   raw;
  float rfsr = readResistance(raw);

  if (csvMode) {
    Serial.print(millis());   Serial.print(',');
    Serial.print(loadKg, 2);  Serial.print(',');
    Serial.print(raw);        Serial.print(',');
    Serial.println(rfsr, 0);                       // -1 => open / no force
    delay(SAMPLE_MS_CSV);
    return;
  }

  // Human-readable bring-up view.
  float v = raw * (V_REF_ADC / ADC_MAX);
  Serial.print(raw);     Serial.print('\t');
  Serial.print(v, 3);    Serial.print('\t');
  if (rfsr < 0) Serial.println("open/no-force");
  else          Serial.println(rfsr, 0);
  delay(SAMPLE_MS_LIVE);
}
