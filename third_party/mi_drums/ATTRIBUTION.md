# Vendored code: Mutable Instruments drum voices

The files under `stmlib/` and `plaits/` in this directory are adapted from
[pichenettes/eurorack](https://github.com/pichenettes/eurorack) (the Mutable
Instruments open-source firmware repo), specifically the `stmlib` submodule
and the `plaits/dsp/` tree, as of the commit cloned on 2026-07-19.

## License

All vendored/adapted files are MIT-licensed:

```
Copyright 20xx Emilie Gillet.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Why vendored rather than a git submodule

The full `eurorack` repo is a large multi-module monorepo; we only need a
handful of files from two of its trees (`stmlib`, `plaits/dsp`), and those
files needed real modifications (see below) to run standalone outside
Plaits' full engine/voice framework. That makes this a fork of specific
files, not a "use as-is" dependency - vendoring with clear provenance is a
better fit than a submodule here.

## What was changed, and why

- **`plaits/dsp/dsp.h`**: `kSampleRate` was a compile-time constant hardcoded
  to 48000Hz. Replaced with a plain (non-const) `extern float` that the
  plugin sets once from `NT_globals.sampleRate` in `construct()`, since the
  disting NT's actual rate shouldn't be assumed.
- **`plaits/dsp/drums/analog_bass_drum.h`, `analog_snare_drum.h`**: removed
  the optional "sustain" (free-running drone) mode and its per-mode
  `SineOscillator` member(s). We don't need that mode for a triggered drum
  machine, and unlike the filter's `FREQUENCY_EXACT` template branches
  (never instantiated by any caller here, so the compiler never emits code
  for them), `sustain` is a plain runtime `bool` - both branches of an
  `if (sustain)` are always compiled in regardless of which one actually
  runs at runtime. The (exact, non-approximated) `SineOscillator` that mode
  used calls real `sinf`, which the disting NT firmware's plugin loader may
  not resolve at load time (we hit exactly this failure mode once already,
  with `log2f`, while building the tuner algorithm) - simplest and safest
  to just not compile that path in at all.
- **`plaits/dsp/drums/hi_hat.h`**: removed `RingModNoise` (an alternate
  "metallic" noise source built on Plaits' `Oscillator` class) and
  `LinearVCA`. We only use the canonical 808-style instantiation,
  `HiHat<SquareNoise, SwingVCA, true, false>` (per
  `plaits/dsp/engine/hi_hat_engine.h`), so there was no reason to vendor an
  unused noise source and its own dependency chain.
- **`plaits/dsp/oscillator/sine_oscillator.h`**: the original reads a
  `lut_sine` table from Plaits' shared `resources.cc` - a large generated
  file backing all 16 of Plaits' engines (wavetables, chord tables, etc.)
  that we have no other use for. `synthetic_bass_drum.h` only needs the
  free, pure-lookup-table `Sine()` helper (not the `SineOscillator`/
  `FastSineOscillator` classes, which pull in an additional
  `stmlib/dsp/rsqrt.h` dependency we also don't need) - so this file now
  contains just that function plus the extracted `lut_sine` data (values
  copied byte-for-byte from upstream), rather than the whole original file.
- **`stmlib/dsp/filter.h`**: added a fallback `#define M_PI` (guarded by
  `#ifndef`) - newlib's `<cmath>` only exposes `M_PI` in non-strict-ANSI
  mode, and this project builds with `-std=c++11` (strict), so the macro
  this file relies on wasn't visible otherwise. No other change.
- **Everything else** (`stmlib.h`, `stmlib/dsp/dsp.h`,
  `units.h`/`units.cc`, `parameter_interpolator.h`, `utils/random.h`,
  `synthetic_bass_drum.h`, `synthetic_snare_drum.h`, `fx/overdrive.h`) is
  unmodified other than path-local include adjustments.

See `../../tuner.cpp` for the earlier, smaller-scale case that established
why avoiding uncommon libm symbols matters here: the firmware's plugin
loader only resolves a limited set of them, and unresolved references fail
the whole plugin load, not just the call site.
