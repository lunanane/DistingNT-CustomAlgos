# Attribution

Source: [pichenettes/eurorack](https://github.com/pichenettes/eurorack), `peaks/`
module (Mutable Instruments "Peaks", `peaks/drums/` - 808-style bass drum,
snare drum, and a sine-FM drum "similar to the BD/SD in Anushri"). Commit
`08460a69` (same clone used for `third_party/mi_elements/`).

MIT License - Copyright 2013 Emilie Gillet. Full text:

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

- `peaks/drums/excitation.h` - the exponential-decay excitation used by all
  three voices, vendored verbatim (no LUTs, pure fixed-point arithmetic).
- `peaks/drums/svf.h` - the fixed-point resonant filter used by the bass drum
  and snare drum's resonant bodies, vendored as-is (not replaced with this
  project's existing float `stmlib::Svf`) since its exact tuning curve (via
  `lut_svf_cutoff`/`lut_svf_damp`) is part of the authentic 808-style sound,
  and those LUTs are small (~257 entries each).
- `peaks/drums/bass_drum.{h,cc}` / `snare_drum.{h,cc}` - ported with two
  changes: (1) `Process()` takes a one-shot `trigger` bool instead of a
  per-sample `GateFlags*` array (this project's convention throughout - none
  of its voices hold a sustained gate) and writes directly to a `float*`
  buffer (dividing the original int16_t sample by 32768) instead of int16_t;
  (2) `set_frequency()` no longer does the original's narrow transposition
  around a fixed base note - it takes the target note directly in the same
  Q7 "MIDI note << 7" format used by this project's `kMidiNoteFreq[]`/Pitch
  parameter, giving full-range pitch control instead of Peaks' original
  narrow (roughly +-7 semitone) range.
- `peaks/drums/fm_drum.{h,cc}` - ported with the above two changes, plus:
  - `Morph()`/`Configure()`/`bd_map`/`sd_map`/`sd_range_` dropped entirely -
    those implement a 2D-pad "morph between two 10-point preset lists"
    control scheme for Peaks' own 2-knob panel; this project exposes direct
    Pitch/Release/FM-amount/Character knobs instead, so the individual
    setters (already present in the original class) are used directly.
  - The audio oscillator itself (`wav_sine`, a ~1024-entry interpolated sine
    table + uint32_t phase) is replaced with this project's already-vendored
    `plaits::Sine()` LUT and a plain float 0..1 phase - avoids a second sine
    table for no audible difference. The FM/AM/auxiliary *envelope* shaping
    (`lut_env_expo`, `lut_env_increments`) is kept as the original fixed-point
    LUTs, since that exponential-decay shape is this drum's actual character,
    not something to substitute a generic envelope for.
  - `wav_overdrive` (a second ~1024-entry waveshaping LUT) is replaced with a
    cheap analytic soft-clip (rational tanh-style approximation, no libm
    call) - a secondary "grit" control, not the core of the sound.
  - Pitch-to-frequency conversion no longer uses Peaks' own fixed-point
    `lut_oscillator_increments` table; `ComputeFrequency()` in `fm_drum.cc`
    instead linearly interpolates within a caller-supplied MIDI-note-to-Hz
    table (this project's own `kMidiNoteFreq[]`) - avoids vendoring a second
    pitch LUT, at the cost of linear (rather than exact exponential)
    interpolation *within* a semitone, inaudible for a drum voice.
- `peaks/resources.{h,cc}` - only `lut_svf_cutoff`, `lut_svf_damp`,
  `lut_env_increments`, `lut_env_expo` (the 4 tables the above code actually
  uses), extracted verbatim (byte-for-byte) from the original
  `peaks/resources.cc`. `wav_sine`, `wav_overdrive`, `lut_oscillator_increments`,
  and the `bd_map`/`sd_map` preset tables are NOT included - see above for why
  each was substituted instead of ported.
- `stmlib/utils/dsp.h` (added to the *existing* `third_party/mi_drums/stmlib/`
  tree, not duplicated here) - only the `uint16_t` overload of
  `Interpolate824()` (the fixed-point LUT interpolator both `Svf` classes use
  for `lut_svf_cutoff`/`lut_svf_damp`) - the original file has `int16_t`/
  `uint8_t` overloads and several other functions (`Crossfade`,
  `Interpolate88`, etc.) not used by anything in this project.

## Not vendored

Peaks' high-hat (`peaks/drums/high_hat.h`) was considered but not used - it
has a *fixed* frequency (no pitch control at all, just 6 hardcoded phase
increments), a worse fit for this project's per-slot Pitch knob than Braids'
own cymbal/hi-hat model (`third_party/mi_braids/`), which is genuinely
pitch-controllable. See `third_party/mi_braids/ATTRIBUTION.md`.
