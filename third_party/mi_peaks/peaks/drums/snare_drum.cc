// Copyright 2013 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Modification: one-shot trigger + float output - see snare_drum.h. Uses
// this project's already-vendored stmlib::Random::GetSample() (identical
// fixed-point LCG noise source already used elsewhere, e.g. Peaks' own
// FmDrum and this project's DJ-filter noise) instead of porting a second
// copy of it.

#include "peaks/drums/snare_drum.h"

#include "stmlib/utils/random.h"

namespace peaks {

void SnareDrum::Init() {
  excitation_1_up_.Init();
  excitation_1_up_.set_delay(0);
  excitation_1_up_.set_decay(1536);

  excitation_1_down_.Init();
  excitation_1_down_.set_delay((uint16_t)(1e-3 * 48000));
  excitation_1_down_.set_decay(3072);

  excitation_2_.Init();
  excitation_2_.set_delay((uint16_t)(1e-3 * 48000));
  excitation_2_.set_decay(1200);

  excitation_noise_.Init();
  excitation_noise_.set_delay(0);

  body_1_.Init();
  body_2_.Init();

  noise_.Init();
  noise_.set_resonance(2000);

  set_tone(0);
  set_snappy(32768);
  set_decay(32768);
  set_frequency(52 << 7);
}

void SnareDrum::Process(bool trigger, float* out, size_t size) {
  if (trigger) {
    excitation_1_up_.Trigger(15 * 32768);
    excitation_1_down_.Trigger(-1 * 32768);
    excitation_2_.Trigger(13107);
    excitation_noise_.Trigger(snappy_);
  }

  while (size--) {
    int32_t excitation_1 = 0;
    excitation_1 += excitation_1_up_.Process();
    excitation_1 += excitation_1_down_.Process();
    excitation_1 += !excitation_1_down_.done() ? 2621 : 0;

    int32_t body_1 = body_1_.Process<SVF_MODE_BP>(excitation_1) + (excitation_1 >> 4);

    int32_t excitation_2 = 0;
    excitation_2 += excitation_2_.Process();
    excitation_2 += !excitation_2_.done() ? 13107 : 0;

    int32_t body_2 = body_2_.Process<SVF_MODE_BP>(excitation_2) + (excitation_2 >> 4);
    int32_t noise_sample = stmlib::Random::GetSample();
    int32_t noise = noise_.Process<SVF_MODE_BP>(noise_sample);
    int32_t noise_envelope = excitation_noise_.Process();
    int32_t sd = 0;
    sd += body_1 * gain_1_ >> 15;
    sd += body_2 * gain_2_ >> 15;
    sd += noise_envelope * noise >> 15;
    CLIP(sd);

    *out++ = sd / 32768.0f;
  }
}

}  // namespace peaks
