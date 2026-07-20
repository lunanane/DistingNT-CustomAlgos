// Copyright 2013 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// 808-style snare drum.
//
// Modification: same one-shot-trigger/float-output/direct-Q7-pitch changes
// as bass_drum.h - see its header comment for the rationale.

#ifndef PEAKS_DRUMS_SNARE_DRUM_H_
#define PEAKS_DRUMS_SNARE_DRUM_H_

#include "stmlib/stmlib.h"

#include "peaks/drums/svf.h"
#include "peaks/drums/excitation.h"

namespace peaks {

class SnareDrum {
 public:
  SnareDrum() { }
  ~SnareDrum() { }

  void Init();
  void Process(bool trigger, float* out, size_t size);

  void set_tone(uint16_t tone) {
    gain_1_ = 22000 - (tone >> 2);
    gain_2_ = 22000 + (tone >> 2);
  }

  void set_snappy(uint16_t snappy) {
    snappy >>= 1;
    if (snappy >= 28672) {
      snappy = 28672;
    }
    snappy_ = 512 + snappy;
  }

  void set_decay(uint16_t decay) {
    body_1_.set_resonance(29000 + (decay >> 5));
    body_2_.set_resonance(26500 + (decay >> 5));
    excitation_noise_.set_decay(4092 + (decay >> 14));
  }

  void set_frequency(int32_t base_note_q7) {
    body_1_.set_frequency(base_note_q7);
    body_2_.set_frequency(base_note_q7 + (12 << 7));
    noise_.set_frequency(base_note_q7 + (48 << 7));
  }

 private:
  Excitation excitation_1_up_;
  Excitation excitation_1_down_;
  Excitation excitation_2_;
  Excitation excitation_noise_;
  Svf body_1_;
  Svf body_2_;
  Svf noise_;

  int32_t gain_1_;
  int32_t gain_2_;

  uint16_t snappy_;

  DISALLOW_COPY_AND_ASSIGN(SnareDrum);
};

}  // namespace peaks

#endif  // PEAKS_DRUMS_SNARE_DRUM_H_
