// Copyright 2013 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Modification: one-shot trigger + float output - see bass_drum.h.

#include "peaks/drums/bass_drum.h"

namespace peaks {

void BassDrum::Init() {
  pulse_up_.Init();
  pulse_down_.Init();
  attack_fm_.Init();
  resonator_.Init();

  pulse_up_.set_delay(0);
  pulse_up_.set_decay(3340);

  pulse_down_.set_delay((uint16_t)(1.0e-3 * 48000));
  pulse_down_.set_decay(3072);

  attack_fm_.set_delay((uint16_t)(4.0e-3 * 48000));
  attack_fm_.set_decay(4093);

  resonator_.set_punch(32768);

  set_frequency(31 << 7);
  set_decay(32768);
  set_tone(32768);
  set_punch(65535);
  set_attack_fm_amount(0);
  set_fm_mode(0);

  lp_state_ = 0;
  previous_sample_ = 0;
}

void BassDrum::Process(bool trigger, float accent, float* out, size_t size) {
  if (trigger) {
    pulse_up_.Trigger((int32_t)(12 * 32768 * 0.7));
    pulse_down_.Trigger((int32_t)(-19662 * 0.7));
    attack_fm_.Trigger(18000);
    if (fm_mode_ >= 1) {
      previous_sample_ = (int32_t)(accent * 32767.0f);
    }
  }

  int32_t attack_fm_push = (17 << 7) * attack_fm_amount_ >> 16;
  int32_t previous_sample = previous_sample_;

  while (size--) {
    int32_t excitation = 0;
    excitation += pulse_up_.Process();
    excitation += !pulse_down_.done() ? 16384 : 0;
    excitation += pulse_down_.Process();
    attack_fm_.Process();

    // Envelope-driven push (the original behavior), or genuine audio-rate
    // self-feedback (previous_sample's magnitude modulates pitch directly,
    // in Q15 fixed point), or both - see fm_mode_'s header comment.
    int32_t fm_push = 0;
    if (fm_mode_ == 0 || fm_mode_ == 5) {
      fm_push += attack_fm_.done() ? 0 : attack_fm_push;
    }
    if (fm_mode_ >= 1) {
      int32_t fb = previous_sample;
      int32_t mag = fb < 0 ? -fb : fb;   // already <= 32767, CLIP()'d below
      int32_t sign = fb < 0 ? -1 : 1;
      int32_t shaped = mag;
      if (fm_mode_ == 2) {
        shaped = (int32_t)(((int64_t)mag * mag) >> 15);
      } else if (fm_mode_ == 3) {
        int32_t sq = (int32_t)(((int64_t)mag * mag) >> 15);
        shaped = (int32_t)(((int64_t)sq * mag) >> 15);
      } else if (fm_mode_ == 4) {
        shaped = (int32_t)(((int64_t)mag * (65534 - mag)) >> 15);
      }
      fm_push += (int32_t)(((int64_t)attack_fm_push * sign * shaped) >> 15);
    }
    // Feedback can swing fm_push negative enough to push the resonator's
    // frequency to (or past) 0 at the very top of attack_fm_amount_'s
    // range, which reads as silence - drumMachine.cpp caps the knob-derived
    // amount below that danger zone instead of clamping fm_push directly
    // here (which flattened the knock's character well before the actual
    // danger zone).
    resonator_.set_frequency(frequency_ + fm_push);

    int32_t resonator_output = (excitation >> 4) +
        resonator_.Process<SVF_MODE_BP>(excitation);
    lp_state_ += (resonator_output - lp_state_) * lp_coefficient_ >> 15;
    int32_t output = lp_state_;
    CLIP(output);

    previous_sample = output;
    *out++ = output / 32768.0f;
  }

  previous_sample_ = previous_sample;
}

}  // namespace peaks
