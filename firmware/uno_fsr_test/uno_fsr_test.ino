/*
 * Arduino Uno / classic Nano (ATmega328P) — FSR402 characterization test
 * -----------------------------------------------------------------------
 * Purpose: read ONE FSR402 and report it as both raw ADC and FSR resistance
 *          (ohms). Resistance is the portable quantity — a curve measured here
 *          transfers directly to the XIAO nRF52840 later, even though that
 *          board runs at 3.3 V with a different ADC.
 *
 * Wiring (voltage divider):
 *
 *     VCC ----[ FSR402 ]----+----[ R_FIXED ]---- GND
 *                           |
 *                          A0   (analog input)
 *
 *   As force increases, FSR resistance drops, so the A0 node voltage rises.
 *
 * Notes:
 *   - On the Uno or classic Nano (both ATmega328P @ 5V), power the divider
 *     from 5V and keep V_REF_ADC = 5.0. (The Nano's "3V3" pin is just a
 *     low-current tap from the USB chip — the board still runs at 5V.)
 *   - On a XIAO nRF52840, power the divider from 3.3V and set BOTH constants
 *     to 3.3 (and never exceed 3.3 V on an input pin). The printed resistance
 *     will still be directly comparable across boards.
 *   - Tune R_FIXED: start 10k; lower (~3.3k) if it saturates under sitting
 *     loads, raise for more sensitivity to light contact.
 *
 * Build+upload / monitor (arduino-cli, classic Nano on /dev/ttyUSB0):
 *   arduino-cli compile --upload -p /dev/ttyUSB0 \
 *     --fqbn arduino:avr:nano:cpu=atmega328 \
 *     ~/Development/TCC/firmware/uno_fsr_test
 *   arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200
 */

const int   FSR_PIN     = A0;
const float V_REF_ADC   = 5.0;       // ADC reference (Uno default = 5V)
const float V_DIVIDER   = 5.0;       // supply feeding the divider
const float R_FIXED     = 3300.0;   // fixed resistor, ohms
const int   ADC_MAX     = 1023;      // 10-bit ADC on the Uno

void setup() {
  Serial.begin(115200);
  Serial.println("raw\tV_node\tR_fsr(ohm)");
}

void loop() {
  int   raw = analogRead(FSR_PIN);
  float v   = raw * (V_REF_ADC / ADC_MAX);   // node voltage from ADC counts

  // Divider: v = V_DIVIDER * R_FIXED / (R_fsr + R_FIXED)
  //   =>  R_fsr = R_FIXED * (V_DIVIDER - v) / v
  // v ~ 0  => no/very light force (FSR effectively open).
  float rfsr = (v > 0.05) ? R_FIXED * (V_DIVIDER - v) / v : -1.0;

  Serial.print(raw);
  Serial.print('\t');
  Serial.print(v, 3);
  Serial.print('\t');
  if (rfsr < 0) Serial.println("open/no-force");
  else          Serial.println(rfsr, 0);

  delay(200);
}
