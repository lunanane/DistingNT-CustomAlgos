// Adapted from Mutable Instruments' plaits/dsp/drums/hi_hat.h (MIT license,
// see ../../../ATTRIBUTION.md).
//
// Modification: the original also included `RingModNoise` (an alternate
// "metallic"/KR-55-style noise source built on plaits' `Oscillator` class)
// and `LinearVCA`. We only use the canonical 808-style instantiation -
// `HiHat<SquareNoise, SwingVCA, true, false>`, per plaits/dsp/engine/
// hi_hat_engine.h - so both were removed rather than vendoring the
// `Oscillator` class for a noise source we don't use.
//
// 808 HH, with a few extra parameters to push things to the CY territory...

#ifndef PLAITS_DSP_DRUMS_HI_HAT_H_
#define PLAITS_DSP_DRUMS_HI_HAT_H_

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

#include "plaits/dsp/dsp.h"

namespace plaits {

// 808 style "metallic noise" with 6 square oscillators.
class SquareNoise {
 public:
  SquareNoise() { }
  ~SquareNoise() { }

  void Init() {
    std::fill(&phase_[0], &phase_[6], 0);
  }

  void Render(float f0, float* temp_1, float* temp_2, float* out, size_t size) {
    const float ratios[6] = {
        // Nominal f0: 414 Hz
        1.0f, 1.304f, 1.466f, 1.787f, 1.932f, 2.536f
    };

    uint32_t increment[6];
    uint32_t phase[6];
    for (int i = 0; i < 6; ++i) {
      float f = f0 * ratios[i];
      if (f >= 0.499f) f = 0.499f;
      increment[i] = static_cast<uint32_t>(f * 4294967296.0f);
      phase[i] = phase_[i];
    }

    while (size--) {
      phase[0] += increment[0];
      phase[1] += increment[1];
      phase[2] += increment[2];
      phase[3] += increment[3];
      phase[4] += increment[4];
      phase[5] += increment[5];
      uint32_t noise = 0;
      noise += (phase[0] >> 31);
      noise += (phase[1] >> 31);
      noise += (phase[2] >> 31);
      noise += (phase[3] >> 31);
      noise += (phase[4] >> 31);
      noise += (phase[5] >> 31);
      *out++ = 0.33f * static_cast<float>(noise) - 1.0f;
    }

    for (int i = 0; i < 6; ++i) {
      phase_[i] = phase[i];
    }
  }

 private:
  uint32_t phase_[6];

  DISALLOW_COPY_AND_ASSIGN(SquareNoise);
};

class SwingVCA {
 public:
  float operator()(float s, float gain) {
   s *= s > 0.0f ? 4.0f : 0.1f;
   s = s / (1.0f + fabsf(s));
   return (s + 0.1f) * gain;
  }
};

template<
    typename MetallicNoiseSource,
    typename VCA,
    bool resonance,
    bool two_stage_envelope>
class HiHat {
 public:
  HiHat() { }
  ~HiHat() { }

  void Init() {
    envelope_ = 0.0f;
    noise_clock_ = 0.0f;
    noise_sample_ = 0.0f;

    metallic_noise_.Init();
    noise_coloration_svf_.Init();
    hpf_.Init();
  }

  void Render(
      bool trigger,
      float accent,
      float f0,
      float tone,
      float decay,
      float noisiness,
      float* temp_1,
      float* temp_2,
      float* out,
      size_t size) {
    const float envelope_decay = 1.0f - 0.003f * stmlib::SemitonesToRatio(
        -decay * 84.0f);
    const float cut_decay = 1.0f - 0.0025f * stmlib::SemitonesToRatio(
        -decay * 36.0f);

    if (trigger) {
      envelope_ = (1.5f + 0.5f * (1.0f - decay)) * (0.3f + 0.7f * accent);
    }

    // Render the metallic noise.
    metallic_noise_.Render(2.0f * f0, temp_1, temp_2, out, size);

    // Apply BPF on the metallic noise.
    float cutoff = 150.0f / kSampleRate * stmlib::SemitonesToRatio(
        tone * 72.0f);
    CONSTRAIN(cutoff, 0.0f, 16000.0f / kSampleRate);
    noise_coloration_svf_.set_f_q<stmlib::FREQUENCY_ACCURATE>(
        cutoff, resonance ? 3.0f + 3.0f * tone : 1.0f);
    noise_coloration_svf_.Process<stmlib::FILTER_MODE_BAND_PASS>(
        out, out, size);

    // This is not at all part of the 808 circuit! But to add more variety, we
    // add a variable amount of clocked noise to the output of the 6 schmitt
    // trigger oscillators.
    noisiness *= noisiness;
    float noise_f = f0 * (16.0f + 16.0f * (1.0f - noisiness));
    CONSTRAIN(noise_f, 0.0f, 0.5f);

    for (size_t i = 0; i < size; ++i) {
      noise_clock_ += noise_f;
      if (noise_clock_ >= 1.0f) {
        noise_clock_ -= 1.0f;
        noise_sample_ = stmlib::Random::GetFloat() - 0.5f;
      }
      out[i] += noisiness * (noise_sample_ - out[i]);
    }

    // Apply VCA.
    for (size_t i = 0; i < size; ++i) {
      VCA vca;
      envelope_ *= envelope_ > 0.5f || !two_stage_envelope
          ? envelope_decay
          : cut_decay;
      out[i] = vca(out[i], envelope_);
    }

    hpf_.set_f_q<stmlib::FREQUENCY_ACCURATE>(cutoff, 0.5f);
    hpf_.Process<stmlib::FILTER_MODE_HIGH_PASS>(out, out, size);
  }

 private:
  float envelope_;
  float noise_clock_;
  float noise_sample_;

  MetallicNoiseSource metallic_noise_;
  stmlib::Svf noise_coloration_svf_;
  stmlib::Svf hpf_;

  DISALLOW_COPY_AND_ASSIGN(HiHat);
};

}  // namespace plaits

#endif  // PLAITS_DSP_DRUMS_HI_HAT_H_
