// Copyright 2013 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Resource declarations - only the 4 LUTs actually vendored (see
// resources.cc's comment for what was deliberately left out and why).

#ifndef PEAKS_RESOURCES_H_
#define PEAKS_RESOURCES_H_

#include "stmlib/stmlib.h"

namespace peaks {

extern const uint16_t lut_svf_cutoff[];
extern const uint16_t lut_svf_damp[];
extern const uint32_t lut_env_increments[];
extern const uint16_t lut_env_expo[];

}  // namespace peaks

#endif  // PEAKS_RESOURCES_H_
