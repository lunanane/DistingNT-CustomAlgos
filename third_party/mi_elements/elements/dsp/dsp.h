// Copyright 2014 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Modification: kSampleRate is a runtime-settable extern float (was a
// compile-time 32000.0f constant) - see ATTRIBUTION.md.

#ifndef ELEMENTS_DSP_DSP_H_
#define ELEMENTS_DSP_DSP_H_

#include "stmlib/stmlib.h"

namespace elements {

extern float kSampleRate;
const size_t kMaxBlockSize = 16;

}  // namespace elements

#endif  // ELEMENTS_DSP_DSP_H_
