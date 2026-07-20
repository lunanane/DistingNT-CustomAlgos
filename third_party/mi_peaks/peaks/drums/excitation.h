// Copyright 2013 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Exponential decay excitation - vendored verbatim (fixed-point, no LUTs).

#ifndef PEAKS_DRUMS_EXCITATION_H_
#define PEAKS_DRUMS_EXCITATION_H_

#include "stmlib/stmlib.h"

namespace peaks {

class Excitation {
 public:
  Excitation() { }
  ~Excitation() { }

  void Init() {
    delay_ = 0;
    decay_ = 4093;
    counter_ = 0;
    state_ = 0;
  }

  void set_delay(uint16_t delay) {
    delay_ = delay;
  }

  void set_decay(uint16_t decay) {
    decay_ = decay;
  }

  void Trigger(int32_t level) {
    level_ = level;
    counter_ = delay_ + 1;
  }

  bool done() {
    return counter_ == 0;
  }

  inline int32_t Process() {
    state_ = (state_ * decay_ >> 12);
    if (counter_ > 0) {
      --counter_;
      if (counter_ == 0) {
        state_ += level_ < 0 ? -level_ : level_;
      }
    }
    return level_ < 0 ? -state_ : state_;
  }

 private:
  uint32_t delay_;
  uint32_t decay_;
  int32_t counter_;
  int32_t state_;
  int32_t level_;

  DISALLOW_COPY_AND_ASSIGN(Excitation);
};

}  // namespace peaks

#endif  // PEAKS_DRUMS_EXCITATION_H_
