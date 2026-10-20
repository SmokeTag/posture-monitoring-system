// The board core compiles with -Ofast: floating-point isnan/v!=v checks are
// not a valid oracle here. Assert observable protocol and CSV behavior.
#include "check.h"
#include <PostureLink.h>
#include <SerialOut.h>
#include <string.h>
using namespace posture;

static float fromBits(uint32_t bits) {
  float v;
  memcpy(&v, &bits, sizeof(v));
  return v;
}

int main() {
  const uint32_t nanBits[] = {0x7fc00000u, 0xffc00000u, 0x7f800001u};
  for (uint32_t bits : nanBits) {
    float nan = fromBits(bits);
    Line line;
    line.fix(nan, 1);
    CHECK_STR(line.text(), "nan");
    CHECK(!anglesOk(nan, 0));
    CHECK(!anglesOk(0, nan));

    BodyPacket p = {};
    p.ver = PROTO_VER; p.boot = 0x84; p.age = AGE_NONE;
    p.pitch = p.roll = nan;
    BodyPacket parsed;
    for (uint8_t status = IMU_INIT; status <= IMU_RANGE; status++) {
      p.imu = status;
      CHECK(parseBodyPacket(&p, sizeof(p), parsed));
      p.pitch = 0;
      CHECK(!parseBodyPacket(&p, sizeof(p), parsed));
      p.pitch = nan;
    }
    p.imu = IMU_OK;
    CHECK(!parseBodyPacket(&p, sizeof(p), parsed));

    ChairAck ack = {};
    ack.ver = PROTO_VER; ack.calId = 1; ack.flags = CF_CAL;
    ack.pitchRef = nan;
    ChairAck parsedAck;
    CHECK(!parseChairAck(&ack, sizeof(ack), parsedAck));
  }
  Line positive, negative, zero;
  positive.fix(fromBits(0x7f800000u), 1);
  negative.fix(fromBits(0xff800000u), 1);
  zero.fix(fromBits(0x80000000u), 1);
  CHECK_STR(positive.text(), "inf");
  CHECK_STR(negative.text(), "-inf");
  CHECK_STR(zero.text(), "-0.0");
  CHECK(!anglesOk(fromBits(0x7f800000u), 0));
  CHECK(!anglesOk(0, fromBits(0xff800000u)));
  CHECK(anglesOk(-90, 180));
  CHECK(!anglesOk(90.1f, 0));
  return report("board -Ofast protocol/CSV");
}
