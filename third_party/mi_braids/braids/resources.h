// Copyright 2013 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Resource declarations - only the 2 LUTs actually vendored (Cymbal's own
// Svf cutoff/damp tables - see resources.cc).

#ifndef BRAIDS_RESOURCES_H_
#define BRAIDS_RESOURCES_H_

#include "stmlib/stmlib.h"

namespace braids {

extern const uint16_t lut_svf_cutoff[];
extern const uint16_t lut_svf_damp[];

}  // namespace braids

#endif  // BRAIDS_RESOURCES_H_
