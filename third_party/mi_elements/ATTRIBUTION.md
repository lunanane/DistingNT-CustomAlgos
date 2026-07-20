# Attribution

Source: [pichenettes/eurorack](https://github.com/pichenettes/eurorack), `elements/`
module (Mutable Instruments "Elements", modal synthesis - exciter into
resonator). Commit `08460a69` (cloned 2026-07-19 while researching drum-usable
DSP across the Mutable Instruments catalogue).

MIT License - Copyright 2014 Emilie Gillet. Full text:

```
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
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

## What was vendored

- `elements/dsp/exciter.{h,cc}` - trimmed to the "Mallet" excitation model
  only. The original supports 7 models selected by an enum + a
  pointer-to-member-function dispatch table:
  - `GranularSamplePlayer`/`SamplePlayer` need large embedded sample-data
    LUTs (`smp_noise_sample`, `smp_sample_data`, `smp_boundaries`) - not
    vendored, we don't want that dependency.
  - `Plectrum`/`Particles`/`Flow`/`Noise` are all cheap and plausible future
    additions (see the research note that flagged this module), but left
    out of this first pass to keep it small - only `Mallet` (single filtered
    impulse per trigger) is wired up.
  - `flags` (originally a bitfield covering rising/falling edge + a
    continuous gate, `ExciterFlags`) is reduced to a plain one-shot
    `trigger` bool, matching every other voice in this project - none of
    them hold a sustained gate, so the original's "while gated, do X, else
    decay" branches collapse to "always decay after the trigger sample."
- `elements/dsp/resonator.{h,cc}` - the modal bandpass-bank resonator, with:
  - The "bowed modes" path removed entirely (`f_bow_`/`d_bow_`/
    `bow_signal_`, sustained-bow banded waveguides needing 8
    `stmlib::DelayLine<float,1024>` instances - roughly 32KB, far too large
    for a per-voice budget in a 4-voice drum machine, and not relevant to a
    one-shot struck/mallet voice anyway).
  - The stereo "sides" output removed (only ever added comb-filtered stereo
    width on top of the "center" signal - no audible effect on center
    itself; this project pans an already-mono signal per voice instead,
    same as its other models).
  - `resolution_`/`set_resolution()` (runtime mode-count knob) replaced with
    a fixed `kNumModes` - initially 24 (the original source comments that
    "the first 24 modes are updated every time [at ~2kHz], the higher
    modes... at the slowest rate," i.e. 24 was already treated as the
    perceptually-important threshold), then cut further to **8** after real
    hardware CPU measurements showed one Elements voice costing roughly 2x
    the CPU of any other model on this plugin (~30% vs 12-15%) - the biggest
    single lever since `kNumModes` is literally the count of `stmlib::Svf`
    instances processed per sample per voice. Also serves the user's
    explicit ask for it to sound "more lo-fi" - fewer resonant partials reads
    as more synthetic/less pristine, not just cheaper.
  - `ComputeFilters()` gained a coefficient-recompute cache: it now skips
    recomputing every mode's filter coefficients if (frequency, geometry,
    brightness, damping) are all unchanged since the last block (a voice
    sitting on a static pitch/character between hits, which is most of the
    time) - same pattern as `cachedFilterParam` already used for the DJ
    filter in `drumMachine.cpp`.
  - Per-sample position interpolation and per-sample `CosineOscillator`
    re-`Init()` (there to avoid zipper noise if `position` changes
    abruptly) simplified to once per block - this project's existing
    parameter smoothing (`_paramSmoother` in `drumMachine.cpp`) already
    ramps knob changes smoothly at a higher level, so the extra per-sample
    work wasn't buying anything here.
- `elements/resources.h` - only the 6 LUTs the above two files actually
  reference (`lut_approx_svf_gain/g/r/h`, `lut_4_decades`, `lut_stiffness`),
  extracted verbatim (byte-for-byte) from the original `elements/resources.cc`.
  The original file also has large sample-playback and accent-gain tables
  used only by the excitation models we didn't vendor - not included.
- `elements/dsp/dsp.h` - trimmed to just `kSampleRate`, changed from a
  compile-time `32000.0f` constant to a runtime-settable `extern float`
  (set once from `NT_globals.sampleRate` in `construct()`, same pattern
  already used for `plaits::kSampleRate` - see
  `distingnt-dsp-vendoring-workflow` project memory).
- `stmlib/dsp/cosine_oscillator.h` - copied as-is (needed by `resonator.cc`,
  not previously vendored) into the *existing* `third_party/mi_drums/stmlib/`
  tree rather than duplicating a second `stmlib/` here, since that's already
  an active include path and this is a generic (non-Elements-specific)
  utility from the same shared `stmlib` library. Only the
  `COSINE_OSCILLATOR_APPROXIMATE` template mode is ever instantiated (pure
  polynomial approximation, no libm call); the `COSINE_OSCILLATOR_EXACT`
  branch calls `cosf()` but is never instantiated, so it's never linked in -
  same "unused template branch" safety argument already used elsewhere in
  this project for `FREQUENCY_EXACT`.

## Not vendored (yet)

Peaks' `BassDrum`/`SnareDrum`/`FmDrum` and Braids' `PARTICLE_NOISE` were also
identified as strong drum-voice candidates in the same research pass, but
need meaningfully more adaptation work (Peaks uses fixed-point arithmetic and
a per-sample `GateFlags*` array API throughout, quite different from this
project's float/one-shot-trigger convention; Braids' particle noise needs LUT
extraction from an 11,500-line `resources.cc`) and were deliberately left for
a following pass rather than rushed alongside this one.
