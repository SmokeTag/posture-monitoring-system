// The Seeed core uses -Ofast, which assumes finite floating-point operands.
// Inspect the IEEE-754 representation for protocol validation and CSV output;
// isnan(), isfinite(), v != v and signbit() can be optimized away there.
#pragma once
#include <stdint.h>
#include <string.h>
#include <float.h>

namespace posture {
static_assert(sizeof(float) == sizeof(uint32_t) && FLT_RADIX == 2 &&
              FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128, "binary32 float required");

inline uint32_t floatBits(float v) {
  uint32_t bits;
  memcpy(&bits, &v, sizeof(bits));             // no aliasing or FP arithmetic
  return bits;
}
inline bool floatIsNan(float v) {
  return (floatBits(v) & 0x7fffffffu) > 0x7f800000u;
}
inline bool floatIsFinite(float v) {
  return (floatBits(v) & 0x7f800000u) != 0x7f800000u;
}
}  // namespace posture
