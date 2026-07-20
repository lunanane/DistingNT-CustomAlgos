// Copyright 2013 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Modifications (see fm_drum.h for the interface-level ones):
// - The audio oscillator itself (`wav_sine` interpolated LUT + uint32_t
//   phase) is replaced with this project's already-vendored plaits::Sine()
//   LUT + a plain float 0..1 phase, avoiding a second ~1024-entry sine
//   table. The FM/AM/auxiliary *envelope* shaping (`lut_env_expo`,
//   `lut_env_increments`) is kept as the original fixed-point LUTs - small
//   (~257 entries each) and doing the exact exponential-decay shape that's
//   this drum's actual character, not just a generic envelope this project
//   could substitute its own for.
// - `wav_overdrive` (a second ~1024-entry waveshaping LUT) is replaced with
//   a cheap analytic soft-clip (rational tanh-style approximation, no libm
//   call) - a secondary "grit" control, not the core of the sound.
// - Pitch-to-frequency conversion no longer uses Peaks' own fixed-point
//   `lut_oscillator_increments` table; ComputeFrequency() below instead
//   linearly interpolates within the caller-supplied `midi_note_freq[]`
//   table (this project's own kMidiNoteFreq[]) - avoids vendoring a second
//   pitch LUT, at the cost of linear (rather than exact exponential)
//   interpolation *within* a semitone, which is inaudible for a drum voice.

#include "peaks/drums/fm_drum.h"

#include "stmlib/utils/dsp.h"
#include "stmlib/utils/random.h"

#include "peaks/resources.h"

#include "plaits/dsp/oscillator/sine_oscillator.h"

namespace peaks {

static float ComputeFrequency(int32_t note_q7, const float* midi_note_freq) {
  if (note_q7 < 0) note_q7 = 0;
  if (note_q7 > (126 << 7)) note_q7 = 126 << 7;
  int32_t note_index = note_q7 >> 7;
  float frac = (note_q7 & 127) * (1.0f / 128.0f);
  return midi_note_freq[note_index] +
      frac * (midi_note_freq[note_index + 1] - midi_note_freq[note_index]);
}

// Cheap rational tanh-style soft clip - stands in for the original's
// wav_overdrive LUT (see file header comment).
static float SoftClip(float x) {
  if (x < -3.0f) x = -3.0f;
  if (x > 3.0f) x = 3.0f;
  return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

void FmDrum::Init() {
  phase_ = 0.0f;
  fm_envelope_phase_ = 0xffffffff;
  am_envelope_phase_ = 0xffffffff;
  aux_envelope_phase_ = 0xffffffff;
  previous_sample_ = 0.0f;
  current_frequency_hz_ = 100.0f;
  set_frequency(60 << 7);
  set_fm_amount(0);
  set_decay(32768);
  set_noise(32768);
}

uint32_t FmDrum::ComputeEnvelopeIncrement(uint16_t decay) {
  uint32_t a = lut_env_increments[decay >> 8];
  uint32_t b = lut_env_increments[(decay >> 8) + 1];
  return a - ((a - b) * (decay & 0xff) >> 8);
}

void FmDrum::Process(bool trigger, const float* midi_note_freq, float sample_rate, float* out, size_t size) {
  uint32_t am_envelope_increment = ComputeEnvelopeIncrement(am_decay_);
  uint32_t fm_envelope_increment = ComputeEnvelopeIncrement(fm_decay_);

  if (trigger) {
    fm_envelope_phase_ = 0;
    am_envelope_phase_ = 0;
    aux_envelope_phase_ = 0;
    // Tiny initial phase offset scaled by fm_amount_, matching the
    // original's own click-avoidance nudge (see file header).
    phase_ = (16383.0f * fm_amount_) / (65536.0f * 4294967296.0f);
  }

  for (size_t i = 0; i < size; ++i) {
    fm_envelope_phase_ += fm_envelope_increment;
    if (fm_envelope_phase_ < fm_envelope_increment) {
      fm_envelope_phase_ = 0xffffffff;
    }
    aux_envelope_phase_ += 4473924;
    if (aux_envelope_phase_ < 4473924) {
      aux_envelope_phase_ = 0xffffffff;
    }

    if ((i & 3) == 0) {
      uint32_t aux_envelope = 65535 - stmlib::Interpolate824(lut_env_expo, aux_envelope_phase_);
      uint32_t fm_envelope = 65535 - stmlib::Interpolate824(lut_env_expo, fm_envelope_phase_);
      int32_t modulated_pitch = frequency_
          + (int32_t)((fm_envelope * fm_amount_) >> 16)
          + (int32_t)((aux_envelope * (uint32_t)aux_envelope_strength_) >> 15)
          + (int32_t)(previous_sample_ * 512.0f);
      current_frequency_hz_ = ComputeFrequency(modulated_pitch, midi_note_freq);
    }

    phase_ += current_frequency_hz_ / sample_rate;
    phase_ -= (int)phase_;

    float mix = plaits::Sine(phase_);
    if (noise_) {
      float noise_sample = stmlib::Random::GetSample() / 32768.0f;
      float balance = noise_ / 65536.0f;
      mix += (noise_sample - mix) * balance;
    }

    am_envelope_phase_ += am_envelope_increment;
    if (am_envelope_phase_ < am_envelope_increment) {
      am_envelope_phase_ = 0xffffffff;
    }
    uint32_t am_envelope = 65535 - stmlib::Interpolate824(lut_env_expo, am_envelope_phase_);
    mix *= am_envelope / 65535.0f;

    if (overdrive_) {
      float overdriven = SoftClip(mix * 3.0f) * (1.0f / 3.0f);
      float balance = overdrive_ / 65536.0f;
      mix += (overdriven - mix) * balance;
    }

    previous_sample_ = mix;
    *out++ = mix;
  }
}

}  // namespace peaks
