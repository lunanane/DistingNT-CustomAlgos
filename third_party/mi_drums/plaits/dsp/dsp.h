// Adapted from Mutable Instruments' plaits/dsp/dsp.h (MIT license, see
// ../../ATTRIBUTION.md). Modification: kSampleRate was a compile-time
// constant hardcoded to 48000Hz; the disting NT's actual rate is queried at
// runtime via NT_globals.sampleRate, so this is now a plain (non-const)
// variable the plugin sets once in construct(). Every drum voice file
// references it by the same bare name, unmodified.

#ifndef PLAITS_DSP_DSP_H_
#define PLAITS_DSP_DSP_H_

#include "stmlib/stmlib.h"

namespace plaits {

extern float kSampleRate;

}  // namespace plaits

#endif  // PLAITS_DSP_DSP_H_
