/*
 * XIAO nRF52840 Sense (body unit) — receive chair FSR packets over ESB
 * --------------------------------------------------------------------
 * ESB receiver for firmware/chair (library: nrf_to_nrf, TMRh20 —
 * `arduino-cli lib install nrf_to_nrf`; runs the RADIO directly, no SoftDevice).
 * Prints one CSV row per received packet; stage 3 adds the IMU + motor here.
 *
 * The library is polled: the ACK is only sent from inside radio.available(),
 * so loop() must call it often (the chair retries for ~20 ms before giving up).
 *
 * Output (prose lines are '#'-prefixed; pandas read_csv(comment='#')):
 *   t_ms,seq,c8,c9,...,c15        (raw ADC counts 0..4095; t_ms = body clock)
 * Once per second: "# rx pkts=N lost=M" (lost = gaps in seq).
 * A CMD_CALIBRATE from the chair prints "# calibrate".
 *
 * Build / upload / monitor (arduino-cli, run from the repo root):
 *   arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense firmware/body
 *   arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p /dev/ttyACM1 firmware/body
 *   arduino-cli monitor -p /dev/ttyACM1 -c baudrate=115200
 */

#include <Adafruit_TinyUSB.h>                  // needed for USB Serial on this core
#include <nrf_to_nrf.h>

// --- ESB link (keep in sync with firmware/chair/chair.ino) ---
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
bool haveSeq = false;
uint16_t lastSeq = 0;

// link stats, reported and reset once per second
uint16_t nPkts = 0, nLost = 0;
unsigned long lastReport = 0;

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // give the USB monitor a moment

  if (!radio.begin()) Serial.println("# radio init FAILED");
  radio.setChannel(RF_CHANNEL);
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, RF_ADDR);
  radio.startListening();                      // RX role

  Serial.println("# body ESB receiver");
  Serial.println("t_ms,seq,c8,c9,c10,c11,c12,c13,c14,c15");
}

void loop() {
  if (radio.available()) {
    ChairPacket pkt;
    uint8_t len = radio.getDynamicPayloadSize();
    radio.read(&pkt, sizeof(pkt));
    if (len == sizeof(pkt)) {
      if (haveSeq) nLost += (uint16_t)(pkt.seq - lastSeq - 1);
      haveSeq = true;
      lastSeq = pkt.seq;
      nPkts++;

      if (pkt.cmd == CMD_CALIBRATE) Serial.println("# calibrate");
      Serial.print(millis()); Serial.print(','); Serial.print(pkt.seq);
      for (int i = 0; i < 8; i++) { Serial.print(','); Serial.print(pkt.raw[i]); }
      Serial.println();
    }
  }

  unsigned long t = millis();
  if (t - lastReport >= 1000) {
    lastReport = t;
    Serial.print("# rx pkts="); Serial.print(nPkts);
    Serial.print(" lost="); Serial.println(nLost);
    nPkts = nLost = 0;
  }
}
