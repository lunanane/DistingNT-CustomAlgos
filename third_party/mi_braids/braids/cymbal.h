// Copyright 2012 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// 6-partial metallic/noise hi-hat/cymbal, extracted from Braids firmware's
// `DigitalOscillator::RenderCymbal()` (added in v1.7 alongside the 808-style
// kick/snare - see ATTRIBUTION.md) into its own standalone class, since this
// project's per-voice architecture doesn't have (or need) the rest of
// Braids' 40+-shape macro-oscillator around it.
//
// Modifications:
// - Pulled out of `DigitalOscillator` (which shares state/Svf/Excitation
//   pools across dozens of unrelated shapes) into its own small class with
//   just the 2 Svf instances and 6-partial phase state this one model uses.
// - Unlike every other voice in this project (and unlike Peaks'
//   BassDrum/SnareDrum/FmDrum, ported alongside this), the original
//   `RenderCymbal` has no envelope/decay of its own - it's a continuously-
//   running texture, gated only by the fact a *different* Braids shape was
//   selected before. This project's ProcessHat() in drumMachine.cpp supplies
//   its own decay envelope (a plain linear ramp, matching the AD family
//   already used elsewhere) and multiplies it onto this class's output,
//   rather than teaching Cymbal itself a new envelope shape.
// - Pitch-to-phase-increment conversion no longer uses Braids' own
//   fixed-point pitch LUT (`ComputePhaseIncrement`/`lut_oscillator_increments`)
//   - see cymbal.cc's `ComputeFrequency()` for why and what replaces it.

#ifndef BRAIDS_CYMBAL_H_
#define BRAIDS_CYMBAL_H_

#include "stmlib/stmlib.h"

#include "braids/svf.h"

namespace braids {

class Cymbal {
 public:
  Cymbal() { }
  ~Cymbal() { }

  void Init();
  // `midi_note_freq` must point to a 128-entry MIDI-note-to-Hz table (this
  // project's own kMidiNoteFreq[]).
  void Process(const float* midi_note_freq, float sample_rate, float* out, size_t size);

  // Q7 "MIDI note << 7" pitch, same convention as this project's other
  // ported voices - combined with a fixed base note the same way the
  // original did (a cymbal/hi-hat's pitch knob nudges brightness around a
  // fixed center rather than tracking 1:1 like a tonal voice).
  inline void set_pitch(int32_t note_q7) {
    pitch_ = note_q7;
  }

  // Sets both Svf stages' cutoff (brightness) - originally Braids'
  // `parameter_[0]`.
  inline void set_tone(uint16_t tone) {
    tone_ = tone;
  }

  // Crossfades between the metallic 6-partial component (0) and the
  // filtered-noise component (max) - originally Braids' `parameter_[1]`.
  inline void set_xfade(uint16_t xfade) {
    xfade_ = xfade;
  }

 private:
  int32_t pitch_;
  uint16_t tone_;
  uint16_t xfade_;

  uint32_t sub_phase_;      // drives the periodic noise-RNG reseed only
  uint32_t hat_phase_[6];
  uint32_t rng_state_;

  Svf svf_bp_;
  Svf svf_hp_;

  DISALLOW_COPY_AND_ASSIGN(Cymbal);
};

}  // namespace braids

#endif  // BRAIDS_CYMBAL_H_
