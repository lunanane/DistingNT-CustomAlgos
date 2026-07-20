// Copyright 2013 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// 808-style bass drum.
//
// Modification: `Process()` takes a single one-shot `trigger` bool instead
// of a per-sample `GateFlags*` array (this project's convention - none of
// its voices hold a sustained gate) and writes directly to a float buffer
// (dividing the original's int16_t sample by 32768 at the point of output)
// instead of int16_t, matching every other voice already in this project.
// `set_frequency()` no longer does the original's narrow +-7-semitone
// transposition around a fixed base note - it now takes the target note
// directly in the same Q7 "MIDI note << 7" format used by
// `kMidiNoteFreq[]`/the rest of this project's Pitch parameter, giving full-
// range pitch control instead of Peaks' original narrow-range design.
// `set_attack_fm_amount()` is new: the original always pushed the resonator's
// frequency up by a fixed 17 semitones (Q7) for the duration of `attack_fm_`
// after a trigger - the classic 808/909 pitch-knock at the attack, but
// hardcoded, not a knob. This project exposes it as its own FM parameter
// (0..65535 scaling that same 17-semitone push), matching how Plaits'
// Analog/Synthetic bass drum's own "attack FM" knobs are wired to FM too.
// `set_fm_mode()` is also new: mode 0 (Envelope) is the push above,
// unchanged; modes 1=Feedback Linear, 2=Feedback Pow2, 3=Feedback Pow3,
// 4=Feedback Log replace it with genuine audio-rate self-feedback - the
// previous rendered sample's *magnitude* (Q15 fixed point, already bounded
// to +-32767 by Process()'s own CLIP), reshaped and scaled back to that
// same full range, pushes the resonator's frequency instead - and mode
// 5=Both layers envelope and linear feedback together. See
// drumMachine.cpp's kFmModeXxx/"FM Mode" page and
// plaits/dsp/drums/analog_bass_drum.h's matching comment for the full
// rationale (magnitude+rescale rather than reshaping the raw signed sample,
// which shrank toward silence instead of just changing character).
// `Process()` also now takes `accent` - purely to seed `previous_sample_`
// with a velocity-scaled kick right at trigger, since there's no real
// output yet to feed back from at that exact instant otherwise.

#ifndef PEAKS_DRUMS_BASS_DRUM_H_
#define PEAKS_DRUMS_BASS_DRUM_H_

#include "stmlib/stmlib.h"

#include "peaks/drums/svf.h"
#include "peaks/drums/excitation.h"

namespace peaks {

class BassDrum {
 public:
  BassDrum() { }
  ~BassDrum() { }

  void Init();
  void Process(bool trigger, float accent, float* out, size_t size);

  void set_frequency(int32_t frequency_q7) {
    frequency_ = frequency_q7;
  }

  void set_decay(uint16_t decay) {
    uint32_t scaled;
    uint32_t squared;
    scaled = 65535 - decay;
    squared = scaled * scaled >> 16;
    scaled = squared * scaled >> 18;
    resonator_.set_resonance(32768 - 128 - scaled);
  }

  void set_tone(uint16_t tone) {
    uint32_t coefficient = tone;
    coefficient = coefficient * coefficient >> 16;
    lp_coefficient_ = 512 + (coefficient >> 2) * 3;
  }

  void set_punch(uint16_t punch) {
    resonator_.set_punch(punch * punch >> 16);
  }

  void set_attack_fm_amount(uint16_t amount) {
    attack_fm_amount_ = amount;
  }

  void set_fm_mode(int mode) {
    fm_mode_ = mode;
  }

 private:
  Excitation pulse_up_;
  Excitation pulse_down_;
  Excitation attack_fm_;
  Svf resonator_;

  int32_t frequency_;
  int32_t lp_coefficient_;
  int32_t lp_state_;
  int32_t attack_fm_amount_;
  int fm_mode_;
  int32_t previous_sample_;

  DISALLOW_COPY_AND_ASSIGN(BassDrum);
};

}  // namespace peaks

#endif  // PEAKS_DRUMS_BASS_DRUM_H_
