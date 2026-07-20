// Copyright 2014 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Modification: the original's "bowed modes" path (sustained-bow banded
// waveguides: `f_bow_`/`d_bow_`/`bow_signal_`, needing 8 `stmlib::DelayLine
// <float,1024>` instances - ~32KB, far too large for a per-voice drum
// budget) is stripped entirely - not needed for a one-shot struck/mallet
// voice. The stereo "sides" output (comb-filtered for width, no audible
// effect on the main "center" signal) is dropped for the same reason our
// other voices are mono-then-panned. `resolution_`/`set_resolution()` is
// replaced with a fixed `kNumModes` (originally 24, cut to 8 after real
// hardware CPU measurements - one voice at 24 modes cost ~2x the CPU of any
// other model on this plugin; 8 modes also reads as more lo-fi/synthetic,
// which was an explicit ask, not just a CPU tradeoff) instead of a
// runtime-configurable mode count, and per-sample position interpolation/
// CosineOscillator re-Init is simplified to once per block (this project's
// existing parameter smoothing already avoids zipper noise from knob
// changes at a higher level - see ATTRIBUTION.md).

#ifndef ELEMENTS_DSP_RESONATOR_H_
#define ELEMENTS_DSP_RESONATOR_H_

#include "stmlib/stmlib.h"
#include "stmlib/dsp/filter.h"

namespace elements {

const size_t kNumModes = 8;

class Resonator {
 public:
  Resonator() { }
  ~Resonator() { }

  void Init();
  void Process(const float* in, float* out, size_t size);

  inline void set_frequency(float frequency) {
    frequency_ = frequency;
  }

  inline void set_geometry(float geometry) {
    geometry_ = geometry;
  }

  inline void set_brightness(float brightness) {
    brightness_ = brightness;
  }

  inline void set_damping(float damping) {
    damping_ = damping;
  }

  inline void set_position(float position) {
    position_ = position;
  }

 private:
  size_t ComputeFilters();

  float frequency_;
  float geometry_;
  float brightness_;
  float position_;
  float damping_;

  // Cached inputs from the last time ComputeFilters() actually recomputed
  // mode coefficients, plus the mode count that produced - skips the whole
  // per-mode coefficient recompute when nothing's changed since last block
  // (a voice sitting on a static pitch/character between hits, which is
  // most of the time). Sentinel -1.0f can never equal a real frequency_
  // (always > 0), forcing the first call to always compute.
  float cachedFrequency_;
  float cachedGeometry_;
  float cachedBrightness_;
  float cachedDamping_;
  size_t cachedNumModes_;

  stmlib::Svf f_[kNumModes];

  DISALLOW_COPY_AND_ASSIGN(Resonator);
};

}  // namespace elements

#endif  // ELEMENTS_DSP_RESONATOR_H_
