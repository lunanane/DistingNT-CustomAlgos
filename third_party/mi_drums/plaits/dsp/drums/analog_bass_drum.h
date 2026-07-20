// Adapted from Mutable Instruments' plaits/dsp/drums/analog_bass_drum.h
// (MIT license, see ../../../ATTRIBUTION.md).
//
// Modification: the original also supported a "sustain" (free-running
// drone) mode using an exact SineOscillator in place of the resonator.
// That mode isn't needed for a triggered drum machine, and unlike the
// filter's FREQUENCY_EXACT template branches (never instantiated, so never
// compiled in), `sustain` is a plain runtime bool - both branches of any
// `if (sustain)` would always be compiled and linked regardless of which
// path actually runs. Removing it entirely avoids pulling in a real `sinf`
// call that the disting NT firmware may not resolve at load time.
//
// Modification: added `fm_mode` to Render() - besides the original's
// envelope-driven `attack_fm_amount` (a one-shot decaying pulse pushing
// pitch up right at the attack, unrelated to the actual output signal),
// `fm_mode` (0=Envelope, 1=Feedback Linear, 2=Feedback Pow2, 3=Feedback
// Pow3, 4=Feedback Log, 5=Both) can select genuine audio-rate self-feedback
// instead - the previous rendered sample's *magnitude*, reshaped and
// rescaled back to the same peak range, modulates pitch (see
// drumMachine.cpp's kFmModeXxx/"FM Mode" page for the full rationale,
// including why magnitude+rescale rather than reshaping the raw signed
// sample directly - that shrank toward silence instead of just changing
// character). `previous_sample_` is seeded with the trigger's own accent at
// trigger time, so feedback modes have a real, velocity-scaled kick from
// sample 0 - there's no output yet to feed back from at that exact instant
// otherwise, which made the very first version of this feature inaudible.
//
// 808 bass drum model, revisited.

#ifndef PLAITS_DSP_DRUMS_ANALOG_BASS_DRUM_H_
#define PLAITS_DSP_DRUMS_ANALOG_BASS_DRUM_H_

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"

#include "plaits/dsp/dsp.h"

namespace plaits {

class AnalogBassDrum {
 public:
  AnalogBassDrum() { }
  ~AnalogBassDrum() { }

  void Init() {
    pulse_remaining_samples_ = 0;
    fm_pulse_remaining_samples_ = 0;
    pulse_ = 0.0f;
    pulse_height_ = 0.0f;
    pulse_lp_ = 0.0f;
    fm_pulse_lp_ = 0.0f;
    retrig_pulse_ = 0.0f;
    lp_out_ = 0.0f;
    tone_lp_ = 0.0f;
    previous_sample_ = 0.0f;

    resonator_.Init();
  }

  inline float Diode(float x) {
    if (x >= 0.0f) {
      return x;
    } else {
      x *= 2.0f;
      return 0.7f * x / (1.0f + fabsf(x));
    }
  }

  void Render(
      bool trigger,
      float accent,
      float f0,
      float tone,
      float decay,
      float attack_fm_amount,
      float self_fm_amount,
      int fm_mode,
      float* out,
      size_t size) {
    const int kTriggerPulseDuration = 1.0e-3f * kSampleRate;
    const int kFMPulseDuration = 6.0e-3f * kSampleRate;
    const float kPulseDecayTime = 0.2e-3f * kSampleRate;
    const float kPulseFilterTime = 0.1e-3f * kSampleRate;
    const float kRetrigPulseDuration = 0.05f * kSampleRate;

    const float scale = 0.001f / f0;
    const float q = 1500.0f * stmlib::SemitonesToRatio(decay * 80.0f);
    const float tone_f = std::min(
        4.0f * f0 * stmlib::SemitonesToRatio(tone * 108.0f),
        1.0f);
    const float exciter_leak = 0.08f * (tone + 0.25f);
    // Approximate peak |output| for this voice - the feedback reshape
    // curves below preserve amplitude at this reference (rather than
    // shrinking toward silence), and it's also the trigger-time seed scale.
    const float kFeedbackRange = 2.0f;

    if (trigger) {
      pulse_remaining_samples_ = kTriggerPulseDuration;
      fm_pulse_remaining_samples_ = kFMPulseDuration;
      pulse_height_ = 3.0f + 7.0f * accent;
      lp_out_ = 0.0f;
      if (fm_mode >= 1) previous_sample_ = accent * kFeedbackRange;
    }

    while (size--) {
      // Q39 / Q40
      float pulse = 0.0f;
      if (pulse_remaining_samples_) {
        --pulse_remaining_samples_;
        pulse = pulse_remaining_samples_ ? pulse_height_ : pulse_height_ - 1.0f;
        pulse_ = pulse;
      } else {
        pulse_ *= 1.0f - 1.0f / kPulseDecayTime;
        pulse = pulse_;
      }

      // C40 / R163 / R162 / D83
      ONE_POLE(pulse_lp_, pulse, 1.0f / kPulseFilterTime);
      pulse = Diode((pulse - pulse_lp_) + pulse * 0.044f);

      // Q41 / Q42
      float fm_pulse = 0.0f;
      if (fm_pulse_remaining_samples_) {
        --fm_pulse_remaining_samples_;
        fm_pulse = 1.0f;
        // C39 / C52
        retrig_pulse_ = fm_pulse_remaining_samples_ ? 0.0f : -0.8f;
      } else {
        // C39 / R161
        retrig_pulse_ *= 1.0f - 1.0f / kRetrigPulseDuration;
      }
      ONE_POLE(fm_pulse_lp_, fm_pulse, 1.0f / kPulseFilterTime);

      // Q43 and R170 leakage
      float punch = 0.7f + Diode(10.0f * lp_out_ - 1.0f);

      // Q43 / R165 - envelope-driven attack knock (the original behavior),
      // or genuine audio-rate self-feedback (previous_sample_'s magnitude
      // modulates pitch), or both - see fm_mode's header comment.
      float attack_fm = 0.0f;
      if (fm_mode == 0 || fm_mode == 5) {
        attack_fm += fm_pulse_lp_ * 1.7f * attack_fm_amount;
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
        attack_fm += sign * shaped * kFeedbackRange * 1.7f * attack_fm_amount;
      }
      // Feedback can swing negative enough to push f to (or through) 0 at
      // the very top of attack_fm_amount's range - rather than clamp here
      // (which flattened/changed the knock's character even at moderate
      // settings), drumMachine.cpp caps the knob-derived attack_fm_amount
      // itself below that danger zone instead, preserving the exact curve
      // shape everywhere it's actually reachable.
      float self_fm = punch * 0.08f * self_fm_amount;
      float f = f0 * (1.0f + attack_fm + self_fm);
      CONSTRAIN(f, 0.0f, 0.4f);

      float resonator_out;
      resonator_.set_f_q<stmlib::FREQUENCY_DIRTY>(f, 1.0f + q * f);
      resonator_.Process<stmlib::FILTER_MODE_BAND_PASS,
                         stmlib::FILTER_MODE_LOW_PASS>(
          (pulse - retrig_pulse_ * 0.2f) * scale,
          &resonator_out,
          &lp_out_);

      ONE_POLE(tone_lp_, pulse * exciter_leak + resonator_out, tone_f);

      *out++ = tone_lp_;
      previous_sample_ = tone_lp_;
    }
  }

 private:
  int pulse_remaining_samples_;
  int fm_pulse_remaining_samples_;
  float pulse_;
  float pulse_height_;
  float pulse_lp_;
  float fm_pulse_lp_;
  float retrig_pulse_;
  float lp_out_;
  float tone_lp_;
  float previous_sample_;

  stmlib::Svf resonator_;

  DISALLOW_COPY_AND_ASSIGN(AnalogBassDrum);
};

}  // namespace plaits

#endif  // PLAITS_DSP_DRUMS_ANALOG_BASS_DRUM_H_
