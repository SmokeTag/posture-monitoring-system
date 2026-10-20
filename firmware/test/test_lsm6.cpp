// Host unit tests for firmware/libraries/PostureLink/Lsm6Accel.h against the
// fake I2C bus + LSM6DS3TR-C of stub/Wire.h: init (WHO_AM_I, config written
// and read back), checked reads (I2C errors, short reads, a lying count, no
// new data, implausible words) and recovery after a sensor reset. Outputs
// must stay untouched on every failure.
#include "check.h"
#include "stub/Wire.h"
#include <Lsm6Accel.h>

#include <math.h>

using namespace posture;

namespace sim { float ax = 0, ay = 0, az = 1; }
FakeWire Wire1;

static const float SENT = 123.0f;              // "not written" marker

struct Out {
  float x = SENT, y = SENT, z = SENT;
  bool untouched() const { return x == SENT && y == SENT && z == SENT; }
};

// A fresh bus + sensor, begun; optionally with an initialized reader.
static void fresh(FakeWire &w) {
  w = FakeWire();
  w.begin();
  sim::ax = 0; sim::ay = 0; sim::az = 1;
}

static void testBeginOk() {
  FakeWire w; fresh(w);
  w.regs[lsm6::CTRL2_G] = 0x6C;                // gyro left on by older firmware (sensor stayed powered)
  Lsm6Accel<FakeWire> imu(w);
  CHECK_EQ(imu.begin(), IMU_OK);
  CHECK_EQ(imu.whoAmI, 0x6A);
  CHECK_EQ(w.regs[lsm6::CTRL1_XL], 0x66);
  CHECK_EQ(w.regs[lsm6::CTRL2_G], 0x00);
  CHECK_EQ(w.regs[lsm6::CTRL3_C], 0x44);
  CHECK(w.tx + w.rx <= 7);                     // bounded: 5 transactions (2 reads = 4 ops, 3 writes)
  CHECK_EQ(w.available(), 0);
  CHECK_EQ(w.unbegun, 0);
  // begin() again on a configured sensor is harmless
  CHECK_EQ(imu.begin(), IMU_OK);
}

static void testBeginWrongChip() {
  const uint8_t ids[] = {0x69, 0x00, 0xFF, 0x6B};   // plain LSM6DS3, stuck low/high, other
  for (uint8_t id : ids) {
    FakeWire w; fresh(w);
    w.regs[lsm6::WHO_AM_I] = id;
    Lsm6Accel<FakeWire> imu(w);
    CHECK_EQ(imu.begin(), IMU_INIT);
    CHECK_EQ(imu.whoAmI, id);
    CHECK_EQ(w.regWrites, 0);                  // nothing configured on an unknown chip
  }
}

static void testBeginBusErrors() {
  {  // nobody at the address
    FakeWire w; fresh(w);
    w.nackAll = true;
    Lsm6Accel<FakeWire> imu(w);
    CHECK_EQ(imu.begin(), IMU_BUS);
  }
  {  // wrong address configured: NACK, not "success with junk"
    FakeWire w; fresh(w);
    Lsm6Accel<FakeWire> imu(w, 0x6B);
    CHECK_EQ(imu.begin(), IMU_BUS);
  }
  {  // WHO_AM_I read comes back empty / short-but-claimed-full
    FakeWire w; fresh(w);
    w.shortReg = lsm6::WHO_AM_I; w.shortGive = 0;
    Lsm6Accel<FakeWire> imu(w);
    CHECK_EQ(imu.begin(), IMU_BUS);
    CHECK_EQ(imu.whoAmI, 0);                   // the Seeed driver would compare garbage here
    w.lieCount = true;
    CHECK_EQ(imu.begin(), IMU_BUS);
    CHECK_EQ(w.available(), 0);
  }
  {  // config write NACKed
    FakeWire w; fresh(w);
    w.nackData = true;
    Lsm6Accel<FakeWire> imu(w);
    CHECK_EQ(imu.begin(), IMU_BUS);
  }
  {  // tx buffer refuses bytes
    FakeWire w; fresh(w);
    w.writeFails = true;
    Lsm6Accel<FakeWire> imu(w);
    CHECK_EQ(imu.begin(), IMU_BUS);
  }
  {  // short readback of the config block
    FakeWire w; fresh(w);
    w.shortReg = lsm6::CTRL1_XL; w.shortGive = 2;
    Lsm6Accel<FakeWire> imu(w);
    CHECK_EQ(imu.begin(), IMU_BUS);
    w.lieCount = true;
    CHECK_EQ(imu.begin(), IMU_BUS);
    CHECK_EQ(w.available(), 0);
  }
  {  // bus never begun (the real TWIM would be off): an error, not a sample
    FakeWire w = FakeWire();
    Lsm6Accel<FakeWire> imu(w);
    CHECK_EQ(imu.begin(), IMU_BUS);
    CHECK(w.unbegun > 0);
  }
}

static void testBeginReadback() {
  const uint8_t regs[] = {lsm6::CTRL1_XL, lsm6::CTRL2_G, lsm6::CTRL3_C};
  for (uint8_t r : regs) {
    FakeWire w; fresh(w);
    w.regs[lsm6::CTRL2_G] = 0x6C;              // so a stuck CTRL2_G differs from 0x00 too
    w.stuckReg = r;                            // ACKed, but the value doesn't stick
    Lsm6Accel<FakeWire> imu(w);
    CHECK_EQ(imu.begin(), IMU_INIT);
  }
}

static void testReadOk() {
  FakeWire w; fresh(w);
  Lsm6Accel<FakeWire> imu(w);
  CHECK_EQ(imu.begin(), IMU_OK);
  const float v[][3] = {{0, 0, 1}, {-0.342f, 0, 0.94f}, {0.5f, -0.5f, -0.707f}, {1.2f, 0.3f, 0.1f}};
  for (auto &a : v) {
    sim::ax = a[0]; sim::ay = a[1]; sim::az = a[2];
    Out o;
    int tx0 = w.tx, rx0 = w.rx;
    CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_OK);
    CHECK(fabsf(o.x - a[0]) < 0.0005f && fabsf(o.y - a[1]) < 0.0005f && fabsf(o.z - a[2]) < 0.0005f);
    CHECK_EQ(w.tx - tx0, 2);                   // status + data: bounded
    CHECK_EQ(w.rx - rx0, 2);
  }
  // exact decoding, little endian, signed
  w.useRaw = true;
  w.raw[0] = -2049; w.raw[1] = 300; w.raw[2] = 32766;
  Out o;
  CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_OK);
  CHECK(o.x == -2049 * lsm6::G_PER_LSB && o.y == 300 * lsm6::G_PER_LSB && o.z == 32766 * lsm6::G_PER_LSB);
  // a still body: the same counts over and over are fresh samples, not stale
  w.raw[0] = 10; w.raw[1] = -20; w.raw[2] = 2049;
  for (int i = 0; i < 50; i++) {
    Out p;
    CHECK_EQ(imu.read(p.x, p.y, p.z), IMU_OK);
    CHECK(p.z == 2049 * lsm6::G_PER_LSB);
  }
}

static void testReadStale() {
  FakeWire w; fresh(w);
  Lsm6Accel<FakeWire> imu(w);
  CHECK_EQ(imu.begin(), IMU_OK);
  w.forceStale = true;
  Out o;
  int rx0 = w.rx;
  CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_STALE);
  CHECK(o.untouched());
  CHECK_EQ(w.rx - rx0, 1);                     // output registers not even read
  w.forceStale = false;
  w.statusOverride = 0x06;                     // gyro/temp ready, accelerometer not
  CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_STALE);
  CHECK(o.untouched());
  w.statusOverride = -1;
  // never begun: the sensor is in power-down, no sample is ever new
  FakeWire u; fresh(u);
  Lsm6Accel<FakeWire> raw(u);
  CHECK_EQ(raw.read(o.x, o.y, o.z), IMU_STALE);
  CHECK(o.untouched());
}

static void testReadBus() {
  FakeWire w; fresh(w);
  Lsm6Accel<FakeWire> imu(w);
  CHECK_EQ(imu.begin(), IMU_OK);
  Out o;
  w.nackAll = true;
  CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_BUS);
  CHECK(o.untouched());
  w.nackAll = false;
  // status read short
  w.shortReg = lsm6::STATUS_REG; w.shortGive = 0;
  CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_BUS);
  CHECK(o.untouched());
  // data burst short, honest count and lying count, at every length 0..5
  w.shortReg = lsm6::OUTX_L_XL;
  for (uint8_t g = 0; g < 6; g++) {
    for (int lie = 0; lie < 2; lie++) {
      w.shortGive = g; w.lieCount = lie;
      CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_BUS);
      CHECK(o.untouched());
      CHECK_EQ(w.available(), 0);              // nothing left over for the next read
    }
  }
  w.shortReg = -1; w.lieCount = false;
  w.writeFails = true;                         // can't even send the register address
  CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_BUS);
  CHECK(o.untouched());
  w.writeFails = false;
  // and it reads again once the bus is fine
  CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_OK);
}

static void testReadRange() {
  FakeWire w; fresh(w);
  Lsm6Accel<FakeWire> imu(w);
  CHECK_EQ(imu.begin(), IMU_OK);
  w.useRaw = true;
  const int16_t bad[][3] = {
    {0, 0, 0},                                 // all-zero word
    {-1, -1, -1},                              // all-ones word
    {290, 290, 290},                           // |a| 502 LSB < 0.25 g: motion / free fall
    {32767, 0, 0}, {0, -32768, 0}, {0, 0, 32767},   // full scale
  };
  for (auto &r : bad) {
    memcpy(w.raw, r, sizeof w.raw);
    Out o;
    CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_RANGE);
    CHECK(o.untouched());
  }
  // just over 0.25 g and just under full scale: plausible
  const int16_t okv[][3] = {{512, 0, 0}, {0, 0, -512}, {32766, 0, 0}, {-32767, 0, 0}};
  for (auto &r : okv) {
    memcpy(w.raw, r, sizeof w.raw);
    Out o;
    CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_OK);
  }
  // largest norm can't overflow the check
  w.raw[0] = w.raw[1] = w.raw[2] = -32767;
  Out o;
  CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_OK);
}

// The sensor alone resets (brownout): back to power-down, XLDA stops. read()
// says STALE (not an old sample), begin() again restores it.
static void testSensorResetAndRecovery() {
  FakeWire w; fresh(w);
  Lsm6Accel<FakeWire> imu(w);
  CHECK_EQ(imu.begin(), IMU_OK);
  Out o;
  CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_OK);
  w.reset();
  Out s;
  for (int i = 0; i < 5; i++) CHECK_EQ(imu.read(s.x, s.y, s.z), IMU_STALE);
  CHECK(s.untouched());
  CHECK_EQ(imu.begin(), IMU_OK);
  CHECK_EQ(w.regs[lsm6::CTRL3_C], 0x44);
  CHECK_EQ(imu.read(s.x, s.y, s.z), IMU_OK);
  // a sensor that disappears mid-run and comes back
  w.nackAll = true;
  CHECK_EQ(imu.read(s.x, s.y, s.z), IMU_BUS);
  CHECK_EQ(imu.begin(), IMU_BUS);
  w.nackAll = false;
  w.reset();
  CHECK_EQ(imu.begin(), IMU_OK);
  CHECK_EQ(imu.read(s.x, s.y, s.z), IMU_OK);
  // IF_INC lost (a reset leaves it set; force it off): begin() restores and verifies it
  w.regs[lsm6::CTRL3_C] = 0x00;
  CHECK_EQ(imu.begin(), IMU_OK);
  CHECK_EQ(w.regs[lsm6::CTRL3_C], 0x44);
}

// The real Wire1 global is what the body uses: same type, same API.
static void testGlobalBus() {
  fresh(Wire1);
  Lsm6Accel<TwoWire> imu(Wire1);
  CHECK_EQ(imu.begin(), IMU_OK);
  Out o;
  CHECK_EQ(imu.read(o.x, o.y, o.z), IMU_OK);
  CHECK(fabsf(o.z - 1) < 0.0005f);
}

int main() {
  testBeginOk();
  testBeginWrongChip();
  testBeginBusErrors();
  testBeginReadback();
  testReadOk();
  testReadStale();
  testReadBus();
  testReadRange();
  testSensorResetAndRecovery();
  testGlobalBus();
  return report("Lsm6Accel");
}
