// Copyright 2014 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Modification: bow-mode and stereo-sides paths stripped, fixed 8-mode
// count, single per-block amplitude weighting instead of per-sample, and a
// coefficient-recompute cache - see resonator.h for the full rationale.

#include "elements/dsp/resonator.h"

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/cosine_oscillator.h"

#include "elements/dsp/dsp.h"
#include "elements/resources.h"

namespace elements {

void Resonator::Init() {
  for (size_t i = 0; i < kNumModes; ++i) {
    f_[i].Init();
  }
  set_frequency(220.0f / kSampleRate);
  set_geometry(0.25f);
  set_brightness(0.5f);
  set_damping(0.3f);
  set_position(0.999f);
  cachedFrequency_ = -1.0f;
  cachedGeometry_ = -1.0f;
  cachedBrightness_ = -1.0f;
  cachedDamping_ = -1.0f;
  cachedNumModes_ = 0;
}

size_t Resonator::ComputeFilters() {
  if (frequency_ == cachedFrequency_ && geometry_ == cachedGeometry_ &&
      brightness_ == cachedBrightness_ && damping_ == cachedDamping_) {
    return cachedNumModes_;
  }
  cachedFrequency_ = frequency_;
  cachedGeometry_ = geometry_;
  cachedBrightness_ = brightness_;
  cachedDamping_ = damping_;

  float stiffness = stmlib::Interpolate(lut_stiffness, geometry_, 256.0f);
  float harmonic = frequency_;
  float stretch_factor = 1.0f;
  float q = 500.0f * stmlib::Interpolate(lut_4_decades, damping_ * 0.8f, 256.0f);
  float brightness_attenuation = 1.0f - geometry_;
  // Reduces the range of brightness when geometry is very low, to prevent
  // clipping.
  brightness_attenuation *= brightness_attenuation;
  brightness_attenuation *= brightness_attenuation;
  brightness_attenuation *= brightness_attenuation;
  float brightness = brightness_ * (1.0f - 0.2f * brightness_attenuation);
  float q_loss = brightness * (2.0f - brightness) * 0.85f + 0.15f;
  float q_loss_damping_rate = geometry_ * (2.0f - geometry_) * 0.1f;
  size_t num_modes = 0;
  for (size_t i = 0; i < kNumModes; ++i) {
    float partial_frequency = harmonic * stretch_factor;
    if (partial_frequency >= 0.49f) {
      partial_frequency = 0.49f;
    } else {
      num_modes = i + 1;
    }
    f_[i].set_f_q<stmlib::FREQUENCY_FAST>(
        partial_frequency,
        1.0f + partial_frequency * q);
    stretch_factor += stiffness;
    if (stiffness < 0.0f) {
      // Make sure that the partials do not fold back into negative frequencies.
      stiffness *= 0.93f;
    } else {
      // This helps adding a few extra partials in the highest frequencies.
      stiffness *= 0.98f;
    }
    // This prevents the highest partials from decaying too fast.
    q_loss += q_loss_damping_rate * (1.0f - q_loss);
    harmonic += frequency_;
    q *= q_loss;
  }
  cachedNumModes_ = num_modes;
  return num_modes;
}

void Resonator::Process(const float* in, float* out, size_t size) {
  size_t num_modes = ComputeFilters();

  stmlib::CosineOscillator amplitudes;
  amplitudes.Init<stmlib::COSINE_OSCILLATOR_APPROXIMATE>(position_);

  while (size--) {
    amplitudes.Start();
    float input = *in++ * 0.125f;
    float sum_center = 0.0f;
    for (size_t i = 0; i < num_modes; i++) {
      float s = f_[i].Process<stmlib::FILTER_MODE_BAND_PASS>(input);
      sum_center += s * amplitudes.Next();
    }
    *out++ = sum_center;
  }
}

}  // namespace elements
