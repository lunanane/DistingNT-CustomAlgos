// Copyright 2014 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Modification: trimmed to the "Mallet" excitation model only (the original
// has 7 models - GranularSamplePlayer/SamplePlayer need large sample-data
// LUTs we don't want to vendor; Plectrum/Particles/Flow/Noise are cheap but
// left out for now to keep the first Elements integration pass small - see
// ATTRIBUTION.md). No model enum/dispatch table needed as a result.
//
// Excites a single impulse per trigger, filtered by an SVF lowpass whose
// cutoff is set by `timbre` - this becomes the resonator's input.

#ifndef ELEMENTS_DSP_EXCITER_H_
#define ELEMENTS_DSP_EXCITER_H_

#include "stmlib/stmlib.h"
#include "stmlib/dsp/filter.h"

namespace elements {

class Exciter {
 public:
  Exciter() { }
  ~Exciter() { }

  void Init();

  inline void set_parameter(float parameter) {
    parameter_ = parameter;
  }

  inline void set_timbre(float timbre) {
    timbre_ = timbre;
  }

  inline float damping() const {
    return damping_;
  }

  // `trigger` marks the block containing the strike (rising edge) -
  // matches this project's one-shot trigger convention rather than the
  // original's continuous gate-flags byte.
  void Process(bool trigger, float* out, size_t size);

 private:
  float GetPulseAmplitude(float cutoff);

  float parameter_;
  float timbre_;

  stmlib::Svf lp_;
  float damp_state_;
  float damping_;

  DISALLOW_COPY_AND_ASSIGN(Exciter);
};

}  // namespace elements

#endif  // ELEMENTS_DSP_EXCITER_H_
