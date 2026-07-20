// Copyright 2013 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// MIT-licensed - see ATTRIBUTION.md in this directory.
//
// Exponential decay excitation - vendored verbatim (fixed-point, no LUTs).
// Byte-identical to peaks::Excitation (third_party/mi_peaks) - both modules
// happen to share this exact class; kept as a separate copy in its own
// namespace rather than a cross-vendor-directory dependency, matching how
// every other vendored module in this project is self-contained.

#ifndef BRAIDS_EXCITATION_H_
#define BRAIDS_EXCITATION_H_

#include "stmlib/stmlib.h"

namespace braids {

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

}  // namespace braids

#endif  // BRAIDS_EXCITATION_H_
