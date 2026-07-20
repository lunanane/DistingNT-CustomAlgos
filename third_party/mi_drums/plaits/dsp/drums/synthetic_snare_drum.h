// Copyright 2016 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Naive snare drum model (two modulated oscillators + filtered noise).
// Uses a few magic numbers taken from the 909 schematics:
// - Ratio between the two modes of the drum set to 1.47.
// - Funky coupling between the two modes.
// - Noise coloration filters and envelope shapes for the snare.
//
// Modification: added `fm_mode` to Render() - besides the original's
// envelope-driven `fm_` (starts at 1.0 on trigger, decays, modulates pitch -
// unrelated to the actual output signal), `fm_mode` (0=Envelope, 1=Feedback
// Linear, 2=Feedback Pow2, 3=Feedback Pow3, 4=Feedback Log, 5=Both) can
// select genuine audio-rate self-feedback instead - the previous rendered
// sample's *magnitude*, reshaped and rescaled back to the same peak range,
// modulates pitch (see drumMachine.cpp's kFmModeXxx/"FM Mode" page, and
// analog_bass_drum.h's matching comment, for the full rationale).
// `previous_sample_` is seeded with the trigger's own accent at trigger
// time so feedback modes have a real kick from sample 0.

#ifndef PLAITS_DSP_DRUMS_SYNTHETIC_SNARE_DRUM_H_
#define PLAITS_DSP_DRUMS_SYNTHETIC_SNARE_DRUM_H_

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"

#include "plaits/dsp/dsp.h"

namespace plaits {

class SyntheticSnareDrum {
 public:
  SyntheticSnareDrum() { }
  ~SyntheticSnareDrum() { }

  void Init() {
    phase_[0] = 0.0f;
    phase_[1] = 0.0f;
    drum_amplitude_ = 0.0f;
    snare_amplitude_ = 0.0f;
    fm_ = 0.0f;
    hold_counter_ = 0;
    sustain_gain_ = 0.0f;
    previous_sample_ = 0.0f;

    drum_lp_.Init();
    snare_hp_.Init();
    snare_lp_.Init();
  }
  
  inline float DistortedSine(float phase) {
    float triangle = (phase < 0.5f ? phase : 1.0f - phase) * 4.0f - 1.3f;
    return 2.0f * triangle / (1.0f + fabsf(triangle));
  }
  
  void Render(
      bool sustain,
      bool trigger,
      float accent,
      float f0,
      float fm_amount,
      float decay,
      float snappy,
      int fm_mode,
      float* out,
      size_t size) {
    const float decay_xt = decay * (1.0f + decay * (decay - 1.0f));
    fm_amount *= fm_amount;
    const float drum_decay = 1.0f - 1.0f / (0.015f * kSampleRate) * \
        stmlib::SemitonesToRatio(
           -decay_xt * 72.0f - fm_amount * 12.0f + snappy * 7.0f);
    const float snare_decay = 1.0f - 1.0f / (0.01f * kSampleRate) * \
        stmlib::SemitonesToRatio(-decay * 60.0f - snappy * 7.0f);
    const float fm_decay = 1.0f - 1.0f / (0.007f * kSampleRate);
    
    snappy = snappy * 1.1f - 0.05f;
    CONSTRAIN(snappy, 0.0f, 1.0f);
    
    const float drum_level = stmlib::Sqrt(1.0f - snappy);
    const float snare_level = stmlib::Sqrt(snappy);
    
    const float snare_f_min = std::min(10.0f * f0, 0.5f);
    const float snare_f_max = std::min(35.0f * f0, 0.5f);

    snare_hp_.set_f<stmlib::FREQUENCY_FAST>(snare_f_min);
    snare_lp_.set_f_q<stmlib::FREQUENCY_FAST>(snare_f_max,
        0.5f + 2.0f * snappy);
    drum_lp_.set_f<stmlib::FREQUENCY_FAST>(3.0f * f0);
    // See analog_bass_drum.h's kFeedbackRange for the full rationale.
    const float kFeedbackRange = 1.5f;

    if (trigger) {
      snare_amplitude_ = drum_amplitude_ = 0.3f + 0.7f * accent;
      fm_ = 1.0f;
      phase_[0] = phase_[1] = 0.0f;
      hold_counter_ = static_cast<int>((0.04f + decay * 0.03f) * kSampleRate);
      if (fm_mode >= 1) previous_sample_ = accent * kFeedbackRange;
    }
    
    stmlib::ParameterInterpolator sustain_gain(
        &sustain_gain_,
        accent * decay,
        size);
    while (size--) {
      if (sustain) {
        snare_amplitude_ = sustain_gain.Next();
        drum_amplitude_ = snare_amplitude_;
        fm_ = 0.0f;
      } else {
        // Compute all D envelopes.
        // The envelope for the drum has a very long tail.
        // The envelope for the snare has a "hold" stage which lasts between
        // 40 and 70 ms
        drum_amplitude_ *= (drum_amplitude_ > 0.03f || !(size & 1))
            ? drum_decay
            : 1.0f;
        if (hold_counter_) {
          --hold_counter_;
        } else {
          snare_amplitude_ *= snare_decay;
        }
        fm_ *= fm_decay;
      }

      // The 909 circuit has a funny kind of oscillator coupling - the signal
      // leaving Q40's collector and resetting all oscillators allow some
      // intermodulation.
      float reset_noise = 0.0f;
      float reset_noise_amount = (0.125f - f0) * 8.0f;
      CONSTRAIN(reset_noise_amount, 0.0f, 1.0f);
      reset_noise_amount *= reset_noise_amount;
      reset_noise_amount *= fm_amount;
      reset_noise += phase_[0] > 0.5f ? -1.0f : 1.0f;
      reset_noise += phase_[1] > 0.5f ? -1.0f : 1.0f;
      reset_noise *= reset_noise_amount * 0.025f;

      float fm_term = 0.0f;
      if (fm_mode == 0 || fm_mode == 5) {
        fm_term += fm_;
      }
      if (fm_mode >= 1) {
        float fb = previous_sample_;
        CONSTRAIN(fb, -kFeedbackRange, kFeedbackRange);
        float mag = fabsf(fb) * (1.0f / kFeedbackRange);
        float sign = fb < 0.0f ? -1.0f : 1.0f;
        float shaped = mag;
        if (fm_mode == 2) shaped = mag * mag;
        else if (fm_mode == 3) shaped = mag * mag * mag;
        else if (fm_mode == 4) shaped = mag * (2.0f - mag);
        fm_term += sign * shaped * kFeedbackRange;
      }
      // Feedback can swing fm_term negative enough to push f negative at the
      // very top of fm_amount's range - the phase wrap below only handles
      // the >= 1.0f case, not going negative, which would silently break
      // both oscillators. drumMachine.cpp caps the knob-derived fm_amount
      // below that danger zone instead of clamping f directly here (which
      // flattened the knock's character well before the actual danger zone).
      float f = f0 * (1.0f + fm_amount * (4.0f * fm_term));
      phase_[0] += f;
      phase_[1] += f * 1.47f;
      if (reset_noise_amount > 0.1f) {
        if (phase_[0] >= 1.0f + reset_noise) {
          phase_[0] = 1.0f - phase_[0];
        }
        if (phase_[1] >= 1.0f + reset_noise) {
          phase_[1] = 1.0f - phase_[1];
        }
      } else {
        if (phase_[0] >= 1.0f) {
          phase_[0] -= 1.0f;
        }
        if (phase_[1] >= 1.0f) {
          phase_[1] -= 1.0f;
        }
      }
      
      float drum = -0.1f;
      drum += DistortedSine(phase_[0]) * 0.60f;
      drum += DistortedSine(phase_[1]) * 0.25f;
      drum *= drum_amplitude_ * drum_level;
      drum = drum_lp_.Process<stmlib::FILTER_MODE_LOW_PASS>(drum);
      
      float noise = stmlib::Random::GetFloat();
      float snare = snare_lp_.Process<stmlib::FILTER_MODE_LOW_PASS>(noise);
      snare = snare_hp_.Process<stmlib::FILTER_MODE_HIGH_PASS>(snare);
      snare = (snare + 0.1f) * (snare_amplitude_ + fm_) * snare_level;
      
      float mix = snare + drum;  // It's a snare, it's a drum, it's a snare drum.
      *out++ = mix;
      previous_sample_ = mix;
    }
  }

 private:
  float phase_[2];
  float drum_amplitude_;
  float snare_amplitude_;
  float fm_;
  float sustain_gain_;
  int hold_counter_;
  float previous_sample_;
  
  stmlib::OnePole drum_lp_;
  stmlib::OnePole snare_hp_;
  stmlib::Svf snare_lp_;
  
  DISALLOW_COPY_AND_ASSIGN(SyntheticSnareDrum);
};
  
}  // namespace plaits

#endif  // PLAITS_DSP_DRUMS_SYNTHETIC_SNARE_DRUM_H_
