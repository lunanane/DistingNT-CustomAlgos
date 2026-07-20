// Copyright 2014 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Modification: trimmed to the Mallet excitation model only, and `flags`
// (originally a bitfield covering rising/falling edge + a continuous gate)
// reduced to a plain one-shot `trigger` bool - our voices never hold a gate,
// so the original's "if not gated, keep decaying" step now always runs.
// See exciter.h for the rest of the trim rationale.

#include "elements/dsp/exciter.h"

#include <algorithm>

#include "elements/resources.h"

namespace elements {

void Exciter::Init() {
  set_parameter(0.0f);
  set_timbre(0.99f);

  lp_.Init();
  damp_state_ = 0.0f;
  damping_ = 0.0f;
}

float Exciter::GetPulseAmplitude(float cutoff) {
  uint32_t cutoff_index = static_cast<uint32_t>(cutoff * 256.0f);
  if (cutoff_index > 256) cutoff_index = 256;
  return lut_approx_svf_gain[cutoff_index];
}

void Exciter::Process(bool trigger, float* out, size_t size) {
  std::fill(&out[0], &out[size], 0.0f);
  if (trigger) {
    damp_state_ = 0.0f;
    out[0] = GetPulseAmplitude(timbre_);
  }
  damp_state_ = 1.0f - 0.95f * (1.0f - damp_state_);
  damping_ = damp_state_ * (1.0f - parameter_);

  uint32_t cutoff_index = static_cast<uint32_t>(timbre_ * 256.0f);
  if (cutoff_index > 256) cutoff_index = 256;
  lp_.set_g_r_h(lut_approx_svf_g[cutoff_index], 2.0f, lut_approx_svf_h[cutoff_index]);
  lp_.Process<stmlib::FILTER_MODE_LOW_PASS>(out, out, size);
}

}  // namespace elements
