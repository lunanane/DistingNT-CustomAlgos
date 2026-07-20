// Copyright 2012 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md files of the modules that use this.
//
// Fixed-point interpolation helper - only the uint16_t-table overload is
// vendored (the only one used, by peaks::Svf/braids::Svf's lut_svf_cutoff/
// lut_svf_damp lookups); the original stmlib/utils/dsp.h has int16_t/uint8_t
// overloads and several other functions (Crossfade, Interpolate88, etc.)
// not needed by anything in this project.

#ifndef STMLIB_UTILS_DSP_H_
#define STMLIB_UTILS_DSP_H_

#include "stmlib/stmlib.h"

namespace stmlib {

inline uint16_t Interpolate824(const uint16_t* table, uint32_t phase) {
  uint32_t a = table[phase >> 24];
  uint32_t b = table[(phase >> 24) + 1];
  return a + ((b - a) * static_cast<uint32_t>((phase >> 8) & 0xffff) >> 16);
}

}  // namespace stmlib

#endif  // STMLIB_UTILS_DSP_H_
