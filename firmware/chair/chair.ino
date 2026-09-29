/*
 * XIAO nRF52840 Sense (chair unit) — read the 8 FSR402s, send them over ESB
 * -------------------------------------------------------------------------
 * Scans CD74HC4067 channels C8..C15 (one FSR each), reads the shared SIG line
 * on A0, prints one CSV row per scan and transmits the raw counts to the body
 * unit (firmware/body) over Nordic ESB (library: nrf_to_nrf, TMRh20 —
 * `arduino-cli lib install nrf_to_nrf`; runs the RADIO directly, no SoftDevice).
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
 * Send 'c' to queue a calibrate command; it rides on the next packets until
 * one is ACKed. Once per second a '#' line reports the link: packets ACKed,
 * retransmissions, and write->ACK time (an upper bound on one-way latency).
 *
 * Build / upload / monitor (arduino-cli, run from the repo root):
 *   arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense firmware/chair
 *   arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p /dev/ttyACM0 firmware/chair
 *   arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
 */

#include <Adafruit_TinyUSB.h>                  // needed for USB Serial on this core
#include <nrf_to_nrf.h>

const int   SIG_PIN    = A0;
const int   SEL_PINS[4] = {D10, D3, D2, D1};   // S0, S1, S2, S3
const int   FIRST_CH    = 8;                   // FSRs on C8..C15
const int   N_FSR       = 8;

const float R_FIXED     = 3300.0;              // ohms (nominal; use measured if known)
const int   ADC_MAX     = 4095;                // 12-bit
const int   OPEN_RAW    = 8;                   // below this => open / no force (~6 mV)
const int   SETTLE_US   = 50;                  // mux switch + SIG node settling
const int   SAMPLE_MS   = 50;                  // 20 Hz scan rate

// --- ESB link (keep in sync with firmware/body/body.ino) ---
const uint8_t RF_ADDR[6]  = "PCHR1";          // 5-byte pipe address
const uint8_t RF_CHANNEL  = 76;               // 2476 MHz
enum : uint8_t { CMD_NONE = 0, CMD_CALIBRATE = 1 };
struct __attribute__((packed)) ChairPacket {
  uint16_t seq;                                // +1 per scan; gaps = lost packets
  uint8_t  cmd;                                // CMD_*
  uint8_t  reserved;
  uint16_t raw[8];                             // ADC counts, C8..C15
};

nrf_to_nrf radio;
bool rawMode = false;                          // 'r' toggles ohms <-> raw counts
bool pendingCal = false;                       // 'c' -> send CMD_CALIBRATE until ACKed
uint16_t seq = 0;

// link stats, reported and reset once per second
uint16_t nSent = 0, nOk = 0, nRetx = 0;
uint32_t latSum = 0, latMax = 0;
unsigned long lastReport = 0;

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
    } else if (c == 'c' || c == 'C') {
      pendingCal = true;
      Serial.println("# calibrate queued");
    }
  }
}

void sendPacket(const int *raw) {
  ChairPacket pkt;
  pkt.seq = seq++;
  pkt.cmd = pendingCal ? CMD_CALIBRATE : CMD_NONE;
  pkt.reserved = 0;
  for (int i = 0; i < N_FSR; i++) pkt.raw[i] = raw[i];

  unsigned long t0 = micros();
  bool ok = radio.write(&pkt, sizeof(pkt));
  uint32_t dt = micros() - t0;

  nSent++;
  if (ok) {
    nOk++;
    nRetx += radio.getARC();
    latSum += dt;
    if (dt > latMax) latMax = dt;
    if (pkt.cmd == CMD_CALIBRATE) { pendingCal = false; Serial.println("# calibrate sent"); }
  }
}

void reportLink() {
  Serial.print("# tx ok="); Serial.print(nOk); Serial.print('/'); Serial.print(nSent);
  Serial.print(" retx="); Serial.print(nRetx);
  Serial.print(" lat_us avg="); Serial.print(nOk ? latSum / nOk : 0);
  Serial.print(" max="); Serial.println(latMax);
  nSent = nOk = nRetx = 0; latSum = latMax = 0;
}

void setup() {
  Serial.begin(115200);
  for (int b = 0; b < 4; b++) pinMode(SEL_PINS[b], OUTPUT);
  analogReference(AR_VDD4);                    // full scale = VDD -> ratiometric
  analogReadResolution(12);
  analogSampleTime(10);                        // us; source impedance <= R_FIXED

  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // give the USB monitor a moment

  if (!radio.begin()) Serial.println("# radio init FAILED");
  radio.setChannel(RF_CHANNEL);
  radio.enableDynamicPayloads();
  radio.setRetries(5, 15);                     // up to 15 retries, ~1.3 ms apart
  radio.openWritingPipe(RF_ADDR);
  radio.stopListening();                       // TX role

  Serial.println("# chair FSR scan: C8..C15. 'r' = toggle ohms/raw, 'c' = calibrate.");
  printHeader();
}

void loop() {
  pollSerial();

  unsigned long t = millis();
  int raw[N_FSR];
  for (int i = 0; i < N_FSR; i++) raw[i] = readChannel(FIRST_CH + i);
  sendPacket(raw);

  Serial.print(t);
  for (int i = 0; i < N_FSR; i++) {
    Serial.print(',');
    if (rawMode) Serial.print(raw[i]);
    else         Serial.print(rawToOhms(raw[i]), 0);
  }
  Serial.println();

  if (t - lastReport >= 1000) { lastReport = t; reportLink(); }

  long wait = SAMPLE_MS - (long)(millis() - t);
  if (wait > 0) delay(wait);
}
