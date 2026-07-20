// Copyright 2013 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Sine FM drum - similar to the BD/SD in Anushri.
//
// Modifications from the original (see fm_drum.cc for the DSP-level ones):
// - `Morph()`/`Configure()`/`bd_map`/`sd_map`/`sd_range_` dropped entirely -
//   those implement a 2D-pad "morph between two 10-point preset lists"
//   control scheme for Peaks' own 2-knob panel. This project exposes direct
//   Pitch/Release/FM/Character knobs instead, so the individual setters
//   below (already present in the original class) are used directly, and
//   the preset-interpolation machinery (128 more LUT-ish entries) isn't
//   needed at all.
// - One-shot `trigger` bool instead of a per-sample `GateFlags*` array,
//   float output instead of int16_t, same as this project's other ports.
// - `set_frequency()` takes a direct Q7 "MIDI note << 7" value (matching
//   `kMidiNoteFreq[]`/this project's Pitch parameter) instead of Peaks' own
//   narrow 0..65535 mapping - see fm_drum.cc for how the pitch-to-frequency
//   conversion itself was reworked to avoid vendoring Peaks' own fixed-point
//   pitch LUT.

#ifndef PEAKS_DRUMS_FM_DRUM_H_
#define PEAKS_DRUMS_FM_DRUM_H_

#include "stmlib/stmlib.h"

namespace peaks {

class FmDrum {
 public:
  FmDrum() { }
  ~FmDrum() { }

  void Init();
  // `midi_note_freq` must point to a 128-entry MIDI-note-to-Hz table (this
  // project's own kMidiNoteFreq[]) - see fm_drum.cc's ComputeFrequency().
  void Process(bool trigger, const float* midi_note_freq, float sample_rate, float* out, size_t size);

  inline void set_frequency(int32_t note_q7) {
    frequency_ = note_q7;
    // Peaks' own "extra low-end growl" heuristic: the self-FM/auxiliary
    // envelope contributes more at low pitch settings, tapering to 0 by
    // mid-range - reworked to read directly off the Q7 pitch instead of the
    // original's own narrow 0..65535 "frequency" parameter.
    int32_t norm = note_q7 - (24 << 7);
    if (norm < 0) norm = 0;
    if (norm > (72 << 7)) norm = 72 << 7;
    uint32_t frequency16 = (uint32_t)norm * 65535 / (72 << 7);
    if (frequency16 <= 16384) {
      aux_envelope_strength_ = 1024;
    } else if (frequency16 <= 32768) {
      aux_envelope_strength_ = (uint16_t)(2048 - (frequency16 >> 4));
    } else {
      aux_envelope_strength_ = 0;
    }
  }

  inline void set_fm_amount(uint16_t fm_amount) {
    fm_amount_ = fm_amount >> 2;
  }

  inline void set_decay(uint16_t decay) {
    am_decay_ = 16384 + (decay >> 1);
    fm_decay_ = 8192 + (decay >> 2);
  }

  inline void set_noise(uint16_t noise) {
    uint32_t n = noise;
    noise_ = (uint16_t)(noise >= 32768 ? ((n - 32768) * (n - 32768) >> 15) : 0);
    noise_ = (uint16_t)((noise_ >> 2) * 5);
    overdrive_ = (uint16_t)(noise <= 32767 ? ((32767 - n) * (32767 - n) >> 14) : 0);
  }

 private:
  uint32_t ComputeEnvelopeIncrement(uint16_t decay);

  int32_t frequency_;
  uint16_t aux_envelope_strength_;
  uint16_t fm_amount_;
  uint16_t am_decay_;
  uint16_t fm_decay_;
  uint16_t noise_;
  uint16_t overdrive_;
  float previous_sample_;

  float phase_;
  uint32_t fm_envelope_phase_;
  uint32_t am_envelope_phase_;
  uint32_t aux_envelope_phase_;
  float current_frequency_hz_;

  DISALLOW_COPY_AND_ASSIGN(FmDrum);
};

}  // namespace peaks

#endif  // PEAKS_DRUMS_FM_DRUM_H_
