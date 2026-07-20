// Copyright 2012 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Extracted from Braids' RenderCymbal() - see cymbal.h for what changed.

#include "braids/cymbal.h"

namespace braids {

// Reproduces the fractional part of Braids/Peaks' own fixed-point pitch
// LUTs closely enough for a percussive metallic texture (linear
// interpolation *within* a semitone against this project's own
// kMidiNoteFreq[], rather than porting a second exponential pitch table -
// see ATTRIBUTION.md).
static float ComputeFrequency(int32_t note_q7, const float* midi_note_freq) {
  if (note_q7 < 0) note_q7 = 0;
  if (note_q7 > (126 << 7)) note_q7 = 126 << 7;
  int32_t note_index = note_q7 >> 7;
  float frac = (note_q7 & 127) * (1.0f / 128.0f);
  return midi_note_freq[note_index] +
      frac * (midi_note_freq[note_index + 1] - midi_note_freq[note_index]);
}

void Cymbal::Init() {
  svf_bp_.Init();
  svf_bp_.set_mode(SVF_MODE_BP);
  svf_bp_.set_resonance(12000);

  svf_hp_.Init();
  svf_hp_.set_mode(SVF_MODE_HP);
  svf_hp_.set_resonance(2000);

  sub_phase_ = 0;
  for (int i = 0; i < 6; ++i) hat_phase_[i] = 0;
  rng_state_ = 1;

  pitch_ = 60 << 7;
  tone_ = 32768;
  xfade_ = 32768;
}

void Cymbal::Process(const float* midi_note_freq, float sample_rate, float* out, size_t size) {
  int32_t note = (40 << 7) + (pitch_ >> 1);
  float freq0 = ComputeFrequency(note, midi_note_freq);

  uint32_t increments[6];
  increments[0] = (uint32_t)((double)freq0 / sample_rate * 4294967296.0);
  uint32_t root = increments[0] >> 10;
  increments[1] = root * 24273 >> 4;
  increments[2] = root * 12561 >> 4;
  increments[3] = root * 18417 >> 4;
  increments[4] = root * 22452 >> 4;
  increments[5] = root * 31858 >> 4;
  uint32_t sub_increment = increments[0] * 24;

  int32_t xfade = xfade_;
  svf_bp_.set_frequency(tone_ >> 1);
  svf_hp_.set_frequency(tone_ >> 1);

  for (size_t i = 0; i < size; ++i) {
    sub_phase_ += sub_increment;
    if (sub_phase_ < sub_increment) {
      rng_state_ = rng_state_ * 1664525L + 1013904223L;
    }
    for (int p = 0; p < 6; ++p) {
      hat_phase_[p] += increments[p];
    }

    int32_t hat_noise = 0;
    for (int p = 0; p < 6; ++p) {
      hat_noise += hat_phase_[p] >> 31;
    }
    hat_noise -= 3;
    hat_noise *= 5461;
    hat_noise = svf_bp_.Process(hat_noise);
    CLIP(hat_noise)

    int32_t noise = (int32_t)(rng_state_ >> 16) - 32768;
    noise = svf_hp_.Process(noise >> 1);
    CLIP(noise)

    int32_t sample = hat_noise + ((noise - hat_noise) * xfade >> 15);
    *out++ = sample / 32768.0f;
  }
}

}  // namespace braids
