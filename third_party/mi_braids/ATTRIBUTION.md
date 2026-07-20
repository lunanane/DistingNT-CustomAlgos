# Attribution

Source: [pichenettes/eurorack](https://github.com/pichenettes/eurorack),
`braids/` module (Mutable Instruments "Braids", macro-oscillator). Commit
`08460a69` (same clone used for `third_party/mi_elements/`,
`third_party/mi_peaks/`).

The kick/snare/hi-hat models referenced by the user as "Braids firmware 1.9"
(and reproduced by community alternative firmwares like µBraids, which the
user has used directly) were added to the official firmware in **v1.7**
("Added 808 kick and snare models. Added cymbal noise model.") and refined in
**v1.8**. In the source tree, they live in `braids/digital_oscillator.{h,cc}`
as `DigitalOscillator::RenderKick/RenderSnare/RenderCymbal`, not in
`braids/macro_oscillator.{h,cc}` (which only wires up the classic analog-style
waveforms). Comparing them against `peaks/drums/bass_drum.cc` and
`snare_drum.cc` (see `third_party/mi_peaks/ATTRIBUTION.md`) shows the kick and
snare are essentially the *same* 808-style algorithm as Peaks' own
`BassDrum`/`SnareDrum` (same excitation trigger levels, same body-filter
topology) - consistent with the user's own account that these Braids models
"just use[d] Braids 1.9 firmware", itself evidently sharing DNA with Peaks'
drum voices. Only the **cymbal/hi-hat** model is vendored from Braids here;
the kick and snare are covered by the Peaks port instead (cleaner,
already-parameterized `Configure`-style setters, and no reason to vendor the
same algorithm twice).

MIT License - Copyright 2012 Emilie Gillet. Full text:

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

- `braids/excitation.h` - byte-identical to `peaks::Excitation`; kept as its
  own copy in the `braids` namespace rather than a cross-vendor-directory
  dependency (not currently used by `Cymbal`, kept for parity/future use if
  the kick/snare are ever also wanted from this side).
- `braids/svf.h` - the fixed-point resonant filter Cymbal's two filter stages
  use, vendored as-is with Braids' own `lut_svf_cutoff`/`lut_svf_damp`
  (close to, but not byte-identical to, Peaks' tables of the same name - each
  module was tuned independently).
- `braids/cymbal.{h,cc}` - `DigitalOscillator::RenderCymbal()` extracted into
  its own standalone class (this project doesn't have or need the rest of
  Braids' 40+-shape oscillator around it). See cymbal.h's header comment for
  the 3 behavioral changes: no built-in envelope (this project's
  `ProcessHat()` supplies one), and pitch-to-frequency conversion reworked to
  use this project's own `kMidiNoteFreq[]` instead of Braids' own fixed-point
  pitch LUT.
- `braids/resources.{h,cc}` - only `lut_svf_cutoff`/`lut_svf_damp` (the 2
  tables `Cymbal`'s `Svf` uses), extracted verbatim (byte-for-byte) from the
  original `braids/resources.cc`.

## Not vendored

The rest of Braids (`macro_oscillator.*`, `digital_oscillator.*`'s ~35 other
shapes, `analog_oscillator.*`, `quantizer.*`, `settings.*`, `ui.*`) - none of
it is drum-relevant; Braids is fundamentally a melodic macro-oscillator, and
the kick/snare/cymbal models are a small addition within it. `RenderKick`/
`RenderSnare` specifically are not vendored from here at all, in favor of the
Peaks port of the same underlying algorithm (see above).
