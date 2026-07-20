// 4-slot drum machine (bass drum / snare drum / closed hi-hat / open hi-hat)
// built on Mutable Instruments' individual drum-voice DSP classes (vendored
// and trimmed under third_party/mi_drums/ - see ATTRIBUTION.md there), not
// the full Plaits engine, to keep CPU cost far below the built-in Macro
// Oscillator 2 algorithm.
//
// Triggered by MIDI note-on (velocity -> accent). Fully custom UI: the
// standard parameter-page system has no way to tell draw() which page is
// selected, so page tracking, and pot/encoder-to-parameter-value handling,
// are all done here rather than relying on the standard system.

#include <math.h>
#include <string.h>
#include <algorithm>
#include <new>
#include <distingnt/api.h>
#include <distingnt/serialisation.h>
#include <distingnt/wav.h>

#include "third_party/mi_drums/stmlib/stmlib.h"
#include "third_party/mi_drums/stmlib/dsp/dsp.h"
#include "third_party/mi_drums/stmlib/dsp/filter.h"
#include "third_party/mi_drums/stmlib/dsp/units.h"
#include "third_party/mi_drums/stmlib/dsp/parameter_interpolator.h"
#include "third_party/mi_drums/stmlib/utils/random.h"
#include "third_party/mi_drums/plaits/dsp/dsp.h"
#include "third_party/mi_drums/plaits/dsp/oscillator/sine_oscillator.h"
#include "third_party/mi_drums/plaits/dsp/drums/analog_bass_drum.h"
#include "third_party/mi_drums/plaits/dsp/drums/analog_snare_drum.h"
#include "third_party/mi_drums/plaits/dsp/drums/synthetic_bass_drum.h"
#include "third_party/mi_drums/plaits/dsp/drums/synthetic_snare_drum.h"
#include "third_party/mi_drums/plaits/dsp/drums/hi_hat.h"
#include "third_party/mi_drums/plaits/dsp/fx/overdrive.h"

#include "elements/dsp/dsp.h"
#include "elements/dsp/exciter.h"
#include "elements/dsp/resonator.h"

#include "peaks/drums/excitation.h"
#include "peaks/drums/svf.h"
#include "peaks/drums/bass_drum.h"
#include "peaks/drums/snare_drum.h"
#include "peaks/drums/fm_drum.h"

#include "braids/svf.h"
#include "braids/cymbal.h"

// Out-of-line definitions for the vendored code's extern/static members.
// Single-translation-unit build (each plugin .cpp compiles standalone), so
// including units.cc's LUT data directly here is safe - no ODR risk.
float plaits::kSampleRate = 48000.0f;
float elements::kSampleRate = 48000.0f;
uint32_t stmlib::Random::rng_state_ = 1;
#include "third_party/mi_drums/stmlib/dsp/units.cc"
#include "elements/dsp/exciter.cc"
#include "elements/dsp/resonator.cc"
#include "peaks/resources.cc"
#include "peaks/drums/bass_drum.cc"
#include "peaks/drums/snare_drum.cc"
#include "peaks/drums/fm_drum.cc"
#include "braids/resources.cc"
#include "braids/cymbal.cc"

// ---------------------------------------------------------------------
// MIDI note (0-127) -> frequency (Hz) lookup table, equal temperament,
// A4 = note 69 = 440Hz. Avoids any log2/pow call for the Pitch parameter -
// see tuner.cpp for why that matters (the firmware's plugin loader only
// resolves a limited set of libm symbols).
// ---------------------------------------------------------------------
static const float kMidiNoteFreq[128] = {
	8.175799f, 8.661957f, 9.177024f, 9.722718f, 10.300861f, 10.913382f, 11.562326f, 12.249857f,
	12.978272f, 13.750000f, 14.567618f, 15.433853f, 16.351598f, 17.323914f, 18.354048f, 19.445436f,
	20.601722f, 21.826764f, 23.124651f, 24.499715f, 25.956544f, 27.500000f, 29.135235f, 30.867706f,
	32.703196f, 34.647829f, 36.708096f, 38.890873f, 41.203445f, 43.653529f, 46.249303f, 48.999429f,
	51.913087f, 55.000000f, 58.270470f, 61.735413f, 65.406391f, 69.295658f, 73.416192f, 77.781746f,
	82.406889f, 87.307058f, 92.498606f, 97.998859f, 103.826174f, 110.000000f, 116.540940f, 123.470825f,
	130.812783f, 138.591315f, 146.832384f, 155.563492f, 164.813778f, 174.614116f, 184.997211f, 195.997718f,
	207.652349f, 220.000000f, 233.081881f, 246.941651f, 261.625565f, 277.182631f, 293.664768f, 311.126984f,
	329.627557f, 349.228231f, 369.994423f, 391.995436f, 415.304698f, 440.000000f, 466.163762f, 493.883301f,
	523.251131f, 554.365262f, 587.329536f, 622.253967f, 659.255114f, 698.456463f, 739.988845f, 783.990872f,
	830.609395f, 880.000000f, 932.327523f, 987.766603f, 1046.502261f, 1108.730524f, 1174.659072f, 1244.507935f,
	1318.510228f, 1396.912926f, 1479.977691f, 1567.981744f, 1661.218790f, 1760.000000f, 1864.655046f, 1975.533205f,
	2093.004522f, 2217.461048f, 2349.318143f, 2489.015870f, 2637.020455f, 2793.825851f, 2959.955382f, 3135.963488f,
	3322.437581f, 3520.000000f, 3729.310092f, 3951.066410f, 4186.009045f, 4434.922096f, 4698.636287f, 4978.031740f,
	5274.040911f, 5587.651703f, 5919.910763f, 6271.926976f, 6644.875161f, 7040.000000f, 7458.620184f, 7902.132820f,
	8372.018090f, 8869.844191f, 9397.272573f, 9956.063479f, 10548.081821f, 11175.303406f, 11839.821527f, 12543.853951f,
};

static int RoundToInt( float v )
{
	return v >= 0.0f ? (int)( v + 0.5f ) : (int)( v - 0.5f );
}

// Forward declaration - defined near ApplyPost() (its main use), but also
// needed by the SineFold envelope shape corner defined earlier in the file.
static float HardClip( float x );

// ---------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------

enum
{
	kSlotBD, kSlotSD, kSlotCH, kSlotOH,
	kNumSlots,
};
static const char* const kSlotNames[kNumSlots] = { "BD", "SD", "CH", "OH" };

// Modulation target concepts - all 10 smoothed parameters are modulatable via
// the Mod Matrix below; no curation needed since the matrix is a sparse
// "N assignable routes" system, not a dense per-concept grid (an earlier
// dense per-concept-per-slot-per-source parameter block hit an undocumented
// platform limit around 214 total parameters and silently failed to load -
// see kNumModRoutes's comment). kConceptFold is appended at the end (not
// inserted next to kConceptFilter, despite Wavefolder sitting right after
// Filter in the actual signal chain - see ApplyPost()) because Mod Matrix
// routes store a route's target concept as a raw saved integer - inserting
// a new value in the middle would silently repoint any already-saved
// route at the wrong concept, the same bug this caused for the kit
// parameter range (see kParamFoldBD's comment).
enum {
	kConceptRelease, kConceptCompressor, kConceptFilter, kConceptDrive,
	kConceptPitch, kConceptVolume, kConceptTone, kConceptCharacter, kConceptFm,
	kConceptFold,
	kNumConcepts,
};
static_assert( kNumConcepts == 10, "one per smoothed concept" );
static char const * const kEnumModConcept[] = {
	"Release", "Compressor", "Filter", "Waveshape",
	"Pitch", "Volume", "Tone", "Character", "FM",
	"Wavefolder",
};
static_assert( ARRAY_SIZE(kEnumModConcept) == kNumConcepts, "" );

// Mod Matrix: kNumModRoutes assignable routing slots, each a (Source, Target
// Slot, Target Concept, Depth) tuple - replaces an earlier dense per-concept-
// per-slot-per-source parameter grid (144 params for all 9 concepts, cut to a
// curated 64-param 4-concept subset after the full grid hit an undocumented
// platform limit around 214 total parameters). A sparse matrix needs far
// fewer parameters (8 routes x 4 params = 32) *and* restores full modulation
// flexibility (any concept, any slot, no curation) *and* is cheaper at
// runtime - see ModulatedViaMatrix(), which skips unassigned/non-matching
// routes with a cheap integer compare instead of always doing a multiply-add
// per concept per block.
enum { kModSrcNone, kModSrcEnv1, kModSrcEnv2, kModSrcLfo1, kModSrcLfo2, kNumModSources };
static char const * const kEnumModSource[] = { "None", "Env1", "Env2", "LFO1", "LFO2" };
static char const * const kEnumModSlot[] = { "BD", "SD", "CH", "OH" };

enum { kModParamSource, kModParamSlot, kModParamConcept, kModParamDepth, kNumModRouteParams };
// 8 -> 12: the original budget turned out tight in practice (shared across
// every concept/slot/source combination in the whole plugin, not per-page) -
// bumped after confirming the extra 16 params (4 routes x 4) still lands
// comfortably under the ~134 total that's confirmed to load reliably (214
// is confirmed to silently fail - see kFirstModRouteParam's history). Not
// pushed further in one jump since the exact ceiling above 134 is still
// unverified.
enum { kNumModRoutes = 12 };

enum
{
	// Page 0: Routing (Out bus + mode + stereo + pan per slot - see
	// WriteVoiceOutput(). Stereo uses Out as the left bus and Out+1 as
	// right; Pan has no effect on a Mono slot.)
	kParamOutBD, kParamOutBDMode, kParamStereoBD, kParamPanBD,
	kParamOutSD, kParamOutSDMode, kParamStereoSD, kParamPanSD,
	kParamOutCH, kParamOutCHMode, kParamStereoCH, kParamPanCH,
	kParamOutOH, kParamOutOHMode, kParamStereoOH, kParamPanOH,

	// Page 1: MIDI
	kParamMidiMode,
	kParamMidiChannel,
	kParamMidiChBD, kParamMidiChSD, kParamMidiChCH, kParamMidiChOH,
	kParamMidiNoteBD, kParamMidiNoteSD, kParamMidiNoteCH, kParamMidiNoteOH,

	// Page 2: MOD MATRIX (8 routes x 4 params - see ModRouteParam())
	kFirstModRouteParam,
	kLastModRouteParam = kFirstModRouteParam + kNumModRoutes * kNumModRouteParams - 1,

	// Page 3: ENVELOPES (2 generators x Morph/Shape - see AdvanceEnvelope())
	kParamEnv1Morph, kParamEnv1Shape, kParamEnv2Morph, kParamEnv2Shape,

	// Page 4: LFOS (2 LFOs x independent Rate/Shape - see AdvanceLfos())
	kParamLfo1Rate, kParamLfo1Shape, kParamLfo2Rate, kParamLfo2Shape,

	// Page 5: Model
	kParamModelBD, kParamModelSD, kParamModelCH, kParamModelOH,
	// Page 6: Release
	kParamRelBD, kParamRelSD, kParamRelCH, kParamRelOH,
	// Page 7: Compressor (fast/snappy, per-slot amount)
	kParamCompBD, kParamCompSD, kParamCompCH, kParamCompOH,
	// Page 8: Filter (DJ-style bipolar: <0 lowpass, >0 highpass, 0 = bypass)
	kParamFiltBD, kParamFiltSD, kParamFiltCH, kParamFiltOH,
	// Page 9: Waveshaper
	kParamDriveBD, kParamDriveSD, kParamDriveCH, kParamDriveOH,
	// Page 10: Pitch
	kParamPitchBD, kParamPitchSD, kParamPitchCH, kParamPitchOH,
	// Page 11: Volume
	kParamVolBD, kParamVolSD, kParamVolCH, kParamVolOH,
	// Page 12: Tone
	kParamToneBD, kParamToneSD, kParamToneCH, kParamToneOH,
	// Page 13: Character
	kParamCharBD, kParamCharSD, kParamCharCH, kParamCharOH,
	// Page 14: FM - self-contained internal modulation depth (drives the
	// attack pitch-knock on Analog/Synthetic/808, and the core FM amount on
	// the FM model itself - see ProcessKick()/ProcessSnare()), NOT an
	// external audio input. A no-op on Elements and on CH/OH's models (none
	// of which have an equivalent free parameter). Reachable only via the
	// Mod Matrix (see kConceptFm) or this base knob, same as any other
	// concept - matches how Plaits/Braids-style FM works standalone, no
	// patched signal required.
	kParamFmBD, kParamFmSD, kParamFmCH, kParamFmOH,

	// Page 15: WAVEFOLDER AMOUNT - post-fx; applied in ApplyPost() right
	// after Filter and before Waveshaper (see ApplyWavefolder()), even
	// though this parameter block sits here, at the *end* of the smoothed/
	// kit-preset range, not next to Filter/Waveshaper in this enum. It was
	// originally inserted between them for readability, but that shifted
	// every later parameter's saved-preset position by 4 and broke loading
	// old presets (their Volume data landed in the new Pitch slot,
	// clamping to 127 - Volume's range exceeds Pitch's). New persisted
	// params must always be *appended* after everything that already
	// existed when presets were saved, never inserted in the middle -
	// signal-chain order (kept correct - see ApplyPost()) and
	// parameter-table order are independent concerns. Wavefolder Type
	// (page 21, non-modulatable, like Model/FM Mode) picks which fold
	// curve this amount drives.
	kParamFoldBD, kParamFoldSD, kParamFoldCH, kParamFoldOH,

	// Page 16: FM MODE - selects how the FM knob's "knock" energy is
	// generated (see kFmModeXxx below) - a creative/exploratory control, not
	// a modulatable concept (no Mod Matrix entry, like Model).
	kParamFmModeBD, kParamFmModeSD, kParamFmModeCH, kParamFmModeOH,

	// Page 16: SAMPLE - which WAV (if any) from this slot's SD card sample
	// folder (found by name match against kSlotNames - "BD"/"SD"/"CH"/"OH",
	// see ScanSampleFolders()) is layered in. 0 = None (default - the whole
	// sample-mix DSP is skipped entirely when no sample is selected, not
	// just silenced - see MixSampleLayer()). Dynamic .max (0 until the
	// folder's file count is known) - see parameterChanged()/GetSampleDisplayName().
	kParamSampleBD, kParamSampleSD, kParamSampleCH, kParamSampleOH,
	// Page 17: SAMPLE MIX - 0-50% fades the sample in under the synth voice
	// (synth stays full); 50-100% crossfades the synth out until only the
	// sample is heard. See MixSampleLayer().
	kParamSampleMixBD, kParamSampleMixSD, kParamSampleMixCH, kParamSampleMixOH,
	// Page 18: KNOCK/TAIL - a 5-zone performative curve across the loaded
	// sample: 0 = full sample; 0.25 = knock portion only; 0.25-0.5 = a
	// highpass fades in over the knock; 0.5-0.75 = crossfades to the tail
	// portion with the highpass held; 0.75-1.0 = the highpass fades back out
	// leaving the tail portion alone. See MixSampleLayer().
	kParamKnockTailBD, kParamKnockTailSD, kParamKnockTailCH, kParamKnockTailOH,
	// Page 19: MIX TYPE - how the Knock/Tail split point within the sample
	// is found: Fixed (a constant early time window, same for every sample)
	// or Env Follower (adapts per-sample - the point where its amplitude
	// envelope first decays below a threshold after the peak). Both are
	// precomputed once at load time - see AnalyzeSampleSplits().
	kParamMixTypeBD, kParamMixTypeSD, kParamMixTypeCH, kParamMixTypeOH,

	// Page 21: WAVEFOLDER TYPE - which fold curve the Wavefolder Amount
	// knob (kParamFoldBD et al, near the very end of this enum - see its
	// own comment for why) drives - Sine (smooth, LUT-based), Triangle
	// (harder-edged reflect fold), or Buchla (rounder rational soft-fold).
	// Not modulatable (no Mod Matrix entry, like Model/FM Mode/Mix Type) -
	// see kWavefolderTypeXxx below.
	kParamFoldTypeBD, kParamFoldTypeSD, kParamFoldTypeCH, kParamFoldTypeOH,

	kNumParams,
};

// Index into the kFirstModRouteParam..kLastModRouteParam block.
static int ModRouteParam( int route, int which )
{
	return kFirstModRouteParam + route * kNumModRouteParams + which;
}

static char const * const kEnumModelBD[] = { "Analog", "Synthetic", "Elements", "808", "FM" };
static char const * const kEnumModelSD[] = { "Analog", "Synthetic", "Elements", "808", "FM" };
static char const * const kEnumModelHat[] = { "808", "Elements", "Cymbal" };

// FM Mode: selects how the FM knob's attack "knock" energy is actually
// generated on Analog/Synthetic/808. Default adapts per model - Envelope
// for Analog/Synthetic/808 (the one-shot decaying pulse/envelope already
// wired up, unchanged sound), and the FM model's own permanent built-in
// self-modulation blend (it never reads this parameter at all - see
// peaks/drums/fm_drum.cc - so Default is simply a no-op there, matching
// "self-FM" already being its whole character). See ResolveFmMode().
//
// The Feedback-family modes replace the envelope with genuine audio-rate
// self-feedback: the *previous rendered sample's magnitude* (not the raw
// signed sample - see the "no knock" bug below) is reshaped and then
// multiplied back onto the original sign, so the feedback amount always
// spans the model's full 0..1 depth range regardless of which curve is
// picked - only the *timing/shape* of how that depth responds to signal
// level changes, never the ceiling. Pow2/Pow3 are "expander"-like (an
// octave-ish crunch on transients that dies away as the signal itself
// decays, since low-level signal is de-emphasized relative to peaks); Log
// is the opposite - an "ease-up" curve that stays more sensitive at low
// signal levels (the closest analogue to a logarithmic response without an
// actual log() call - see distingnt_libm_symbol_constraints project
// memory). Both layers the envelope and linear feedback together for the
// deepest/most unstable result.
//
// Bug fix: earlier feedback modes fed back the raw sample directly, which
// starts at (or near) 0 right at trigger - there's no signal yet to feed
// back from at the exact instant the knock matters most, so the attack
// was inaudible. Every model seeds `previous_sample_` with the trigger's
// own accent right at trigger time now, so feedback modes have a genuine,
// velocity-scaled kick to work with from sample 0, transitioning to real
// audio-derived feedback as soon as the excitation actually produces sound.
enum {
	kFmModeDefault, kFmModeEnvelope, kFmModeFeedbackLinear,
	kFmModeFeedbackPow2, kFmModeFeedbackPow3, kFmModeFeedbackLog,
	kFmModeBoth, kNumFmModes,
};
static char const * const kEnumFmMode[] = {
	"Default", "Envelope", "Fb Linear", "Fb Pow2", "Fb Pow3", "Fb Log", "Both",
};
static_assert( ARRAY_SIZE(kEnumFmMode) == kNumFmModes, "" );

// Maps the UI-facing kFmModeXxx value (with Default at 0) to the 0-based
// mode number Analog/Synthetic/808's Render()/Process() calls actually
// switch on (0=Envelope..5=Both - see their own header comments) - Default
// resolves to Envelope for these models specifically.
static int ResolveFmMode( int uiMode )
{
	if ( uiMode == kFmModeDefault )
		return 0;	// Envelope
	return uiMode - 1;
}
// Knock/Tail split-point source - see kParamMixTypeXX's comment.
enum { kMixTypeFixed, kMixTypeEnvFollower, kNumMixTypes };
static char const * const kEnumMixType[] = { "Fixed", "Env Follower" };
static_assert( ARRAY_SIZE(kEnumMixType) == kNumMixTypes, "" );

// Wavefolder fold curve - see kParamFoldTypeBD's comment and
// ApplyWavefolder(). All three are libm-safe (LUT or rational-polynomial
// only - no tanh/sinh/atan, which the firmware's plugin loader isn't
// confirmed to resolve, see distingnt_libm_symbol_constraints project
// memory) rather than literal ports of any specific reference circuit.
enum { kWavefolderTypeSine, kWavefolderTypeTriangle, kWavefolderTypeBuchla, kNumWavefolderTypes };
static char const * const kEnumWavefolderType[] = { "Sine", "Triangle", "Buchla" };
static_assert( ARRAY_SIZE(kEnumWavefolderType) == kNumWavefolderTypes, "" );

static char const * const kEnumMidiMode[] = { "Note per slot", "Channel per slot" };
static char const * const kEnumStereo[] = { "Mono", "Stereo" };

// LFO rate: synced clock division, independently selectable per LFO (see
// AdvanceLfos()). kLfoRateMultiplier is in units of quarter notes (a "1/1"
// bar assumes 4/4).
static char const * const kEnumLfoRate[] = { "1/16", "1/8", "1/4", "1/2", "1/1", "2/1", "4/1" };
static const float kLfoRateMultiplier[] = { 0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f, 16.0f };
static_assert( ARRAY_SIZE(kEnumLfoRate) == ARRAY_SIZE(kLfoRateMultiplier), "" );

// One route's 4 parameters - "Rt" + n avoids exceeding typical short-name
// display widths on the standard (non-custom) parameter page list.
#define MOD_ROUTE_PARAMS(n) \
	{ .name = "Rt" #n " source", .min = 0, .max = kNumModSources - 1, .def = kModSrcNone, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumModSource }, \
	{ .name = "Rt" #n " slot", .min = 0, .max = kNumSlots - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumModSlot }, \
	{ .name = "Rt" #n " concept", .min = 0, .max = kNumConcepts - 1, .def = kConceptPitch, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumModConcept }, \
	{ .name = "Rt" #n " depth", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },

static _NT_parameter parameters[] = {
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "BD Out", 1, 13 )
	{ .name = "BD stereo", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumStereo },
	{ .name = "BD pan", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "SD Out", 1, 14 )
	{ .name = "SD stereo", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumStereo },
	{ .name = "SD pan", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "CH Out", 1, 15 )
	{ .name = "CH stereo", .min = 0, .max = 1, .def = 1, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumStereo },
	{ .name = "CH pan", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "OH Out", 1, 17 )
	{ .name = "OH stereo", .min = 0, .max = 1, .def = 1, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumStereo },
	{ .name = "OH pan", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },

	{ .name = "MIDI mode", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumMidiMode },
	{ .name = "MIDI channel", .min = 0, .max = 16, .def = 2, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "BD channel", .min = 0, .max = 16, .def = 1, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD channel", .min = 0, .max = 16, .def = 2, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH channel", .min = 0, .max = 16, .def = 3, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH channel", .min = 0, .max = 16, .def = 4, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "BD note", .min = 0, .max = 127, .def = 60, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD note", .min = 0, .max = 127, .def = 61, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH note", .min = 0, .max = 127, .def = 62, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH note", .min = 0, .max = 127, .def = 63, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },

	MOD_ROUTE_PARAMS(1) MOD_ROUTE_PARAMS(2) MOD_ROUTE_PARAMS(3) MOD_ROUTE_PARAMS(4)
	MOD_ROUTE_PARAMS(5) MOD_ROUTE_PARAMS(6) MOD_ROUTE_PARAMS(7) MOD_ROUTE_PARAMS(8)
	MOD_ROUTE_PARAMS(9) MOD_ROUTE_PARAMS(10) MOD_ROUTE_PARAMS(11) MOD_ROUTE_PARAMS(12)

	{ .name = "Env1 morph", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Env1 shape", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Env2 morph", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Env2 shape", .min = 0, .max = 100, .def = 33, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "LFO1 rate", .min = 0, .max = ARRAY_SIZE(kEnumLfoRate) - 1, .def = 2, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumLfoRate },
	{ .name = "LFO1 shape", .min = 0, .max = 100, .def = 33, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "LFO2 rate", .min = 0, .max = ARRAY_SIZE(kEnumLfoRate) - 1, .def = 2, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumLfoRate },
	{ .name = "LFO2 shape", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD model", .min = 0, .max = 4, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumModelBD },
	{ .name = "SD model", .min = 0, .max = 4, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumModelSD },
	{ .name = "CH model", .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumModelHat },
	{ .name = "OH model", .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumModelHat },

	{ .name = "BD release", .min = 0, .max = 100, .def = 40, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD release", .min = 0, .max = 100, .def = 40, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH release", .min = 0, .max = 100, .def = 20, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH release", .min = 0, .max = 100, .def = 60, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD compressor", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD compressor", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH compressor", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH compressor", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD filter", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD filter", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH filter", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH filter", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD drive", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD drive", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH drive", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH drive", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD pitch", .min = 0, .max = 127, .def = 32, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD pitch", .min = 0, .max = 127, .def = 48, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH pitch", .min = 0, .max = 127, .def = 60, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH pitch", .min = 0, .max = 127, .def = 64, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD volume", .min = 0, .max = 200, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD volume", .min = 0, .max = 200, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH volume", .min = 0, .max = 200, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH volume", .min = 0, .max = 200, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD tone", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD tone", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH tone", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH tone", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD character", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD character", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH character", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH character", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	// Self-contained internal FM depth (no audio input) - only audible on
	// the Synthetic model choice per slot (see ProcessKick()/ProcessSnare()
	// for exactly which Plaits parameter this drives, and why Analog/HiHat
	// don't respond).
	{ .name = "BD FM", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD FM", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH FM", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH FM", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	// Post-fx (see kParamFoldBD's comment for why this sits here in the
	// parameter table rather than next to Filter/Waveshaper).
	{ .name = "BD wavefolder", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD wavefolder", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH wavefolder", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH wavefolder", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD FM mode", .min = 0, .max = kNumFmModes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumFmMode },
	{ .name = "SD FM mode", .min = 0, .max = kNumFmModes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumFmMode },
	{ .name = "CH FM mode", .min = 0, .max = kNumFmModes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumFmMode },
	{ .name = "OH FM mode", .min = 0, .max = kNumFmModes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumFmMode },

	// Sample: 0 = None, max starts at 0 (only None selectable) until the
	// slot's SD card folder is found and its file count known - see
	// ScanSampleFolders()/parameterChanged(). No static enumStrings (the
	// file list is only known at runtime) - kNT_unitHasStrings + the
	// parameterString() callback display it instead, same pattern as the
	// official distingNT_API samplePlayer.cpp example.
	{ .name = "BD sample", .min = 0, .max = 0, .def = 0, .unit = kNT_unitHasStrings, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD sample", .min = 0, .max = 0, .def = 0, .unit = kNT_unitHasStrings, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH sample", .min = 0, .max = 0, .def = 0, .unit = kNT_unitHasStrings, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH sample", .min = 0, .max = 0, .def = 0, .unit = kNT_unitHasStrings, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD sample mix", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD sample mix", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH sample mix", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH sample mix", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD knock/tail", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD knock/tail", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH knock/tail", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH knock/tail", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD mix type", .min = 0, .max = kNumMixTypes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumMixType },
	{ .name = "SD mix type", .min = 0, .max = kNumMixTypes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumMixType },
	{ .name = "CH mix type", .min = 0, .max = kNumMixTypes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumMixType },
	{ .name = "OH mix type", .min = 0, .max = kNumMixTypes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumMixType },

	{ .name = "BD fold type", .min = 0, .max = kNumWavefolderTypes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumWavefolderType },
	{ .name = "SD fold type", .min = 0, .max = kNumWavefolderTypes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumWavefolderType },
	{ .name = "CH fold type", .min = 0, .max = kNumWavefolderTypes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumWavefolderType },
	{ .name = "OH fold type", .min = 0, .max = kNumWavefolderTypes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumWavefolderType },
};

static const uint8_t pageRouting[]   = { kParamOutBD, kParamOutBDMode, kParamStereoBD, kParamPanBD, kParamOutSD, kParamOutSDMode, kParamStereoSD, kParamPanSD, kParamOutCH, kParamOutCHMode, kParamStereoCH, kParamPanCH, kParamOutOH, kParamOutOHMode, kParamStereoOH, kParamPanOH };
static const uint8_t pageMidi[]      = { kParamMidiMode, kParamMidiChannel, kParamMidiChBD, kParamMidiChSD, kParamMidiChCH, kParamMidiChOH, kParamMidiNoteBD, kParamMidiNoteSD, kParamMidiNoteCH, kParamMidiNoteOH };
static const uint8_t pageEnvelopes[] = { kParamEnv1Morph, kParamEnv1Shape, kParamEnv2Morph, kParamEnv2Shape };
static const uint8_t pageLfos[]      = { kParamLfo1Rate, kParamLfo1Shape, kParamLfo2Rate, kParamLfo2Shape };
static const uint8_t pageModel[]     = { kParamModelBD, kParamModelSD, kParamModelCH, kParamModelOH };
static const uint8_t pageRelease[]   = { kParamRelBD, kParamRelSD, kParamRelCH, kParamRelOH };
static const uint8_t pageComp[]      = { kParamCompBD, kParamCompSD, kParamCompCH, kParamCompOH };
static const uint8_t pageFilter[]    = { kParamFiltBD, kParamFiltSD, kParamFiltCH, kParamFiltOH };
static const uint8_t pageDrive[]     = { kParamDriveBD, kParamDriveSD, kParamDriveCH, kParamDriveOH };
static const uint8_t pagePitch[]     = { kParamPitchBD, kParamPitchSD, kParamPitchCH, kParamPitchOH };
static const uint8_t pageVolume[]    = { kParamVolBD, kParamVolSD, kParamVolCH, kParamVolOH };
static const uint8_t pageTone[]      = { kParamToneBD, kParamToneSD, kParamToneCH, kParamToneOH };
static const uint8_t pageChar[]      = { kParamCharBD, kParamCharSD, kParamCharCH, kParamCharOH };
static const uint8_t pageFm[]        = { kParamFmBD, kParamFmSD, kParamFmCH, kParamFmOH };
static const uint8_t pageFmMode[]    = { kParamFmModeBD, kParamFmModeSD, kParamFmModeCH, kParamFmModeOH };
static const uint8_t pageSample[]    = { kParamSampleBD, kParamSampleSD, kParamSampleCH, kParamSampleOH };
static const uint8_t pageSampleMix[] = { kParamSampleMixBD, kParamSampleMixSD, kParamSampleMixCH, kParamSampleMixOH };
static const uint8_t pageKnockTail[] = { kParamKnockTailBD, kParamKnockTailSD, kParamKnockTailCH, kParamKnockTailOH };
static const uint8_t pageMixType[]   = { kParamMixTypeBD, kParamMixTypeSD, kParamMixTypeCH, kParamMixTypeOH };
static const uint8_t pageFold[]      = { kParamFoldBD, kParamFoldSD, kParamFoldCH, kParamFoldOH };
static const uint8_t pageFoldType[]  = { kParamFoldTypeBD, kParamFoldTypeSD, kParamFoldTypeCH, kParamFoldTypeOH };

// The Mod Matrix's 32 params are contiguous (kFirstModRouteParam..+31) in
// exactly this sequential order already, so this is just that range restated
// as a uint8_t param-index list for the standard/custom page machinery -
// filled once from construct() (BuildModMatrixPage()), no name generation
// needed (unlike the old per-slot-per-concept ENVELOPES grid) since each
// route's 4 params already have static names (see MOD_ROUTE_PARAMS()).
static uint8_t pageModMatrix[kNumModRoutes * kNumModRouteParams];
static void BuildModMatrixPage()
{
	for ( int i=0; i<(int)ARRAY_SIZE(pageModMatrix); ++i )
		pageModMatrix[i] = (uint8_t)( kFirstModRouteParam + i );
}

static const _NT_parameterPage pages[] = {
	{ .name = "Routing", .numParams = ARRAY_SIZE(pageRouting), .params = pageRouting },
	{ .name = "MIDI", .numParams = ARRAY_SIZE(pageMidi), .params = pageMidi },
	{ .name = "Mod Matrix", .numParams = ARRAY_SIZE(pageModMatrix), .params = pageModMatrix },
	{ .name = "Envelopes", .numParams = ARRAY_SIZE(pageEnvelopes), .params = pageEnvelopes },
	{ .name = "LFOs", .numParams = ARRAY_SIZE(pageLfos), .params = pageLfos },
	{ .name = "Model", .numParams = ARRAY_SIZE(pageModel), .params = pageModel },
	{ .name = "Release", .numParams = ARRAY_SIZE(pageRelease), .params = pageRelease },
	{ .name = "Compressor", .numParams = ARRAY_SIZE(pageComp), .params = pageComp },
	{ .name = "Filter", .numParams = ARRAY_SIZE(pageFilter), .params = pageFilter },
	{ .name = "Waveshaper", .numParams = ARRAY_SIZE(pageDrive), .params = pageDrive },
	{ .name = "Pitch", .numParams = ARRAY_SIZE(pagePitch), .params = pagePitch },
	{ .name = "Volume", .numParams = ARRAY_SIZE(pageVolume), .params = pageVolume },
	{ .name = "Tone", .numParams = ARRAY_SIZE(pageTone), .params = pageTone },
	{ .name = "Character", .numParams = ARRAY_SIZE(pageChar), .params = pageChar },
	{ .name = "FM", .numParams = ARRAY_SIZE(pageFm), .params = pageFm },
	{ .name = "FM Mode", .numParams = ARRAY_SIZE(pageFmMode), .params = pageFmMode },
	{ .name = "Sample", .numParams = ARRAY_SIZE(pageSample), .params = pageSample },
	{ .name = "Sample Mix", .numParams = ARRAY_SIZE(pageSampleMix), .params = pageSampleMix },
	{ .name = "Knock/Tail", .numParams = ARRAY_SIZE(pageKnockTail), .params = pageKnockTail },
	{ .name = "Mix Type", .numParams = ARRAY_SIZE(pageMixType), .params = pageMixType },
	{ .name = "Wavefolder", .numParams = ARRAY_SIZE(pageFold), .params = pageFold },
	{ .name = "Wavefolder Type", .numParams = ARRAY_SIZE(pageFoldType), .params = pageFoldType },
};

static const _NT_parameterPages parameterPages = {
	.numPages = ARRAY_SIZE(pages),
	.pages = pages,
};

// Custom-UI page layout: a general per-page-index lookup rather than
// assuming list/graph/bar pages sit in contiguous ranges - needed because
// the desired custom-UI cycle order (Envelopes -> LFOs -> Mod Matrix ->
// Model -> ... -> FM) interleaves a list-style page (Mod Matrix) between
// graph-style pages (Envelopes/LFOs) and bar-style pages (Model onward).
// kFirstCustomPage skips Routing/MIDI in our own page cycle (Encoder L) -
// they're still fully editable via the standard, non-custom parameter menu,
// just not duplicated in this overlay (per user feedback: they're patching/
// config, not sound-shaping, so don't need dedicated custom-UI real estate).
enum {
	kPageRouting, kPageMidi, kPageEnvelopes, kPageLfos, kPageModMatrix,
	kPageModel, kPageRelease, kPageCompressor, kPageFilter,
	kPageWaveshaper,
	kPagePitch, kPageVolume, kPageTone, kPageCharacter, kPageFm, kPageFmMode,
	kPageSample, kPageSampleMix, kPageKnockTail, kPageMixType,
	kPageFold, kPageFoldType,
	kNumPages,
};
enum { kFirstCustomPage = kPageEnvelopes };

enum { kPageTypeList, kPageTypeGraph, kPageTypeBar };
static const int kPageType[kNumPages] = {
	kPageTypeList, kPageTypeList, kPageTypeGraph, kPageTypeGraph, kPageTypeList,
	kPageTypeBar, kPageTypeBar, kPageTypeBar, kPageTypeBar,
	kPageTypeBar,
	kPageTypeBar, kPageTypeBar, kPageTypeBar, kPageTypeBar, kPageTypeBar, kPageTypeBar,
	kPageTypeBar, kPageTypeBar, kPageTypeBar, kPageTypeBar,
	kPageTypeBar, kPageTypeBar,
};
// List pages (Routing/MIDI/Mod Matrix) use these via SetupPageParams()/
// SetupPageItemCount(); graph/bar pages (Envelopes/LFOs/Model..Fold Type) use
// kPageParams directly since they always have exactly 4 params (one per
// pot/encoder control - see customUi()).
static const uint8_t* const kPageParams[kNumPages] = {
	pageRouting, pageMidi, pageEnvelopes, pageLfos, pageModMatrix,
	pageModel, pageRelease, pageComp, pageFilter,
	pageDrive,
	pagePitch, pageVolume, pageTone, pageChar, pageFm, pageFmMode,
	pageSample, pageSampleMix, pageKnockTail, pageMixType,
	pageFold, pageFoldType,
};
static const int kPageItemCount[kNumPages] = {
	(int)ARRAY_SIZE(pageRouting), (int)ARRAY_SIZE(pageMidi), 0, 0, (int)ARRAY_SIZE(pageModMatrix),
	0, 0, 0, 0,
	0,
	0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0,
};
static const char* const kPageNames[kNumPages] = {
	"ROUTING", "MIDI", "ENVELOPES", "LFOS", "MOD MATRIX",
	"MODEL", "RELEASE", "COMPRESSOR", "FILTER",
	"WAVESHAPE",
	"PITCH", "VOLUME", "TONE", "CHARACTER", "FM", "FM MODE",
	"SAMPLE", "SAMPLE MIX", "KNOCK/TAIL", "MIX TYPE",
	"WAVEFOLDER", "FOLD TYPE",
};
static const bool kPageBipolar[kNumPages] = {
	false, false, false, false, false,
	false, false, false, true,
	false,
	false, false, false, false, false, false,
	false, false, false, false,
	false, false,
};
// kConceptXxx for each bar-style page, or -1 for non-bar pages and Model/FM
// Mode/Fold Type/Sample/Mix Type (none of which are modulatable concepts).
static const int kPageConcept[kNumPages] = {
	-1, -1, -1, -1, -1,
	-1, kConceptRelease, kConceptCompressor, kConceptFilter,
	kConceptDrive,
	kConceptPitch, kConceptVolume, kConceptTone, kConceptCharacter, kConceptFm, -1,
	-1, -1, -1, -1,
	kConceptFold, -1,
};

// The 10 concepts below Model get smoothed for the preset-load fade (Model
// itself snaps instantly - "models are loaded directly" per the spec).
// They're contiguous in the parameter enum (kParamRelBD..kParamFoldOH -
// Wavefolder is appended at the very end of this range, after FM, rather
// than sitting next to Filter/Waveshaper where it applies in the actual
// signal chain - see kParamFoldBD's comment for why), so a smoothed
// parameter's shadow-state index is just paramIndex - kFirstSmoothedParam.
enum {
	kFirstSmoothedParam = kParamRelBD,
	kNumSmoothedParams = kParamFoldOH - kParamRelBD + 1,
};
static_assert( kNumSmoothedParams == 40, "expected 10 smoothed concepts x 4 slots" );

// A saved "kit" preset covers Model through Wavefolder (also contiguous) -
// deliberately *not* Routing/MIDI, which are how this instance is patched
// into the rig rather than part of the drum sound, so loading a different
// kit doesn't silently move which bus/MIDI note each slot responds to.
enum {
	kFirstKitParam = kParamModelBD,
	kNumKitParams = kParamFoldOH - kParamModelBD + 1,
};
static_assert( kNumKitParams == 44, "expected 4 model + 40 smoothed params" );

// Second preset block: everything that isn't part of the contiguous kit
// range above - the 8 Envelope/LFO globals, FM Mode (4) and the 16
// Sample-layer params (Sample/Sample Mix/Knock-Tail/Mix Type x4 slots -
// none of these three groups are contiguous with the kit range or each
// other, but this block is just a flat index list, not a range, so that's
// not a requirement), plus the Mod Matrix's 32 contiguous route params
// (see SavePreset()/LoadPreset()/serialise()/deserialise()). All snap
// instantly on preset load like Model, rather than fading in - matches
// this whole block's existing "no fade" treatment.
//
// UNVERIFIED HYPOTHESIS, under test: an earlier version of this session
// had the Sample-layer params here too, and shortly after, saving a preset
// triggered the host's own "preset too complex to load, please report as a
// bug" error on the next load. That was pulled out under the theory that
// the added ~1000 ints of extra JSON payload (16 params x 64 preset slots)
// crossed some host-side size limit. But that save happened shortly after
// a separate, real bug (the Env Follower analysis running an uninterrupted
// O(sample length) scan synchronously in the load callback, since fixed -
// see AdvanceSampleAnalysis()) had reportedly overloaded the device's CPU -
// so the corruption may equally have been the save itself glitching under
// an already-overloaded device, unrelated to payload size at all. Re-added
// here specifically so that can be tested in isolation, now that the CPU
// overload itself is fixed: save/load a preset with real sample data in a
// normal (non-overloaded) session and see whether "too complex" recurs. If
// if it does, the payload-size theory holds and this needs a different fix
// (e.g. not all 64 slots' worth); if not, the CPU overload was the whole
// story and this can just stay as-is.
static const uint8_t kModParams[] = {
	kParamEnv1Morph, kParamEnv1Shape, kParamEnv2Morph, kParamEnv2Shape,
	kParamLfo1Rate, kParamLfo1Shape, kParamLfo2Rate, kParamLfo2Shape,
	kParamFmModeBD, kParamFmModeSD, kParamFmModeCH, kParamFmModeOH,
	kParamSampleBD, kParamSampleSD, kParamSampleCH, kParamSampleOH,
	kParamSampleMixBD, kParamSampleMixSD, kParamSampleMixCH, kParamSampleMixOH,
	kParamKnockTailBD, kParamKnockTailSD, kParamKnockTailCH, kParamKnockTailOH,
	kParamMixTypeBD, kParamMixTypeSD, kParamMixTypeCH, kParamMixTypeOH,
	kParamFoldTypeBD, kParamFoldTypeSD, kParamFoldTypeCH, kParamFoldTypeOH,
};
enum { kNumModRouteParamsTotal = kNumModRoutes * kNumModRouteParams };
enum { kNumModParams = ARRAY_SIZE(kModParams) + kNumModRouteParamsTotal };

// Maps a "mod param slot" index (0..kNumModParams-1) to the actual
// parameter index - the first ARRAY_SIZE(kModParams) come from that list,
// the rest are the contiguous Mod Matrix route block.
static int ModParamAt( int i )
{
	if ( i < (int)ARRAY_SIZE(kModParams) )
		return kModParams[i];
	return kFirstModRouteParam + ( i - (int)ARRAY_SIZE(kModParams) );
}

// ---------------------------------------------------------------------
// Voice DSP state
// ---------------------------------------------------------------------

// Per-voice one-shot envelope runtime state (DTC, in _drumVoicePost) - the
// actual trigger/timing instance; Morph/Shape are shared/global knobs (see
// AdvanceEnvelope()/BlendEnvelopeLevel(), defined later since they don't need
// per-voice state), but each voice runs its own independently-triggered
// instance, so e.g. BD's envelope timing never depends on SD's hits.
// `randomSteps` holds the Random contour family's per-trigger step levels
// (regenerated on each trigger) - unused by the AD/ADSR/SineFold families but
// cheap enough to always carry rather than needing a union.
struct _envelopeState
{
	bool active;
	float elapsed;			// seconds since trigger
	float peak;				// accent-scaled peak, locked in at trigger
	float randomSteps[6];

	void Init()
	{
		active = false;
		elapsed = 0.0f;
		peak = 1.0f;
		for ( int i=0; i<6; ++i )
			randomSteps[i] = 0.0f;
	}
};

struct _drumVoicePost
{
	stmlib::Svf lpFilter;
	stmlib::Svf hpFilter;
	// Reused for both waveshaper stages - stage 1 (0-50%) calls it with a
	// varying drive amount; stage 2 (50-100%) just keeps calling it at a
	// fixed drive of 1.0 (its "model 1, maxed out"), so its own internal
	// smoothing settles there and stays there. See ApplyPost().
	plaits::Overdrive drive;
	float compEnvelope;			// fast peak-follower state for the per-slot compressor
	int samplesUntilIdle;
	bool pendingTrigger;
	float pendingAccent;
	// Last filterParam the SVF coefficients were computed for - recomputing
	// set_f_q() every block regardless of whether the knob moved is wasted
	// work (its FREQUENCY_FAST polynomial approximation isn't free), and it
	// almost never changes block-to-block in practice. Sentinel value can
	// never equal a real filterParam (-100..100), forcing the first block
	// to always compute. See ApplyPost().
	int cachedFilterParam;
	_envelopeState env1;
	_envelopeState env2;

	// FM state for models with no internal per-sample FM hook of their own
	// (Elements, plaits::HiHat, braids::Cymbal) - see ComputeExternalFmPush().
	// A coarser, per-BLOCK approximation of the same Envelope/Feedback
	// family Analog/Synthetic/808 already do per-sample internally.
	float fmEnvLevel;
	float fmEnvElapsed;
	float fmPrevSample;

	// Sample-layer playback state (see MixSampleLayer()) - a continuous
	// one-shot read position (in the loaded sample's own native frame rate,
	// not this project's, so playback resamples via a fractional increment
	// exactly like the official distingNT_API samplePlayer.cpp example),
	// and a dedicated highpass distinct from hpFilter above (that one's
	// owned by ApplyPost()'s DJ-style Filter page - reusing it here would
	// fight over its cachedFilterParam-driven coefficients).
	float samplePos;
	stmlib::Svf sampleHp;

	void Init()
	{
		lpFilter.Init();
		hpFilter.Init();
		drive.Init();
		compEnvelope = 0.0f;
		samplesUntilIdle = 0;
		pendingTrigger = false;
		pendingAccent = 1.0f;
		cachedFilterParam = 9999;
		env1.Init();
		env2.Init();
		fmEnvLevel = 0.0f;
		fmEnvElapsed = 0.0f;
		fmPrevSample = 0.0f;
		samplePos = 0.0f;
		sampleHp.Init();
		// Fixed cutoff (not user-adjustable - Knock/Tail already owns this
		// page's controls), so it's cheap to set once here rather than
		// recomputing set_f_q() every block in MixSampleLayer() the way
		// ApplyPost()'s user-adjustable Filter page has to.
		sampleHp.set_f_q<stmlib::FREQUENCY_FAST>( 700.0f / plaits::kSampleRate, 0.6f );
	}
};

struct _kickVoice : _drumVoicePost
{
	plaits::AnalogBassDrum analog;
	plaits::SyntheticBassDrum synthetic;
	elements::Exciter elementsExciter;
	elements::Resonator elementsResonator;
	peaks::BassDrum peaksBass;
	peaks::FmDrum peaksFm;

	void Init()
	{
		_drumVoicePost::Init();
		analog.Init();
		synthetic.Init();
		elementsExciter.Init();
		elementsResonator.Init();
		peaksBass.Init();
		peaksFm.Init();
	}
};

struct _snareVoice : _drumVoicePost
{
	plaits::AnalogSnareDrum analog;
	plaits::SyntheticSnareDrum synthetic;
	elements::Exciter elementsExciter;
	elements::Resonator elementsResonator;
	peaks::SnareDrum peaksSnare;
	peaks::FmDrum peaksFm;

	void Init()
	{
		_drumVoicePost::Init();
		analog.Init();
		synthetic.Init();
		elementsExciter.Init();
		elementsResonator.Init();
		peaksSnare.Init();
		peaksFm.Init();
	}
};

struct _hatVoice : _drumVoicePost
{
	plaits::HiHat<plaits::SquareNoise, plaits::SwingVCA, true, false> hihat;
	elements::Exciter elementsExciter;
	elements::Resonator elementsResonator;
	braids::Cymbal braidsCymbal;
	float* scratch1;	// [maxFramesPerStep], points into dram
	float* scratch2;	// [maxFramesPerStep], points into dram

	// Unlike every other model, Braids' Cymbal has no envelope of its own
	// (it's a continuously-running texture) - see braids/cymbal.h's header
	// comment. This is this voice's own simple linear decay, driven by the
	// Release knob same as any other model, applied as a post-multiply on
	// Cymbal's output in ProcessHat().
	float cymbalElapsed;
	float cymbalTotalS;
	bool cymbalActive;

	void Init()
	{
		_drumVoicePost::Init();
		hihat.Init();
		elementsExciter.Init();
		elementsResonator.Init();
		braidsCymbal.Init();
		cymbalElapsed = 0.0f;
		cymbalTotalS = 1.0f;
		cymbalActive = false;
	}
};

struct _drumMachineAlgorithm_DTC
{
	_kickVoice kick;
	_snareVoice snare;
	_hatVoice closedHat;
	_hatVoice openHat;
};

// Per-slot SD-card sample state - lives in SRAM (alongside the algorithm
// struct itself). Playback uses the disting NT streaming API
// (NT_streamOpen()/NT_streamRender(), see distingNT_API/examples/
// sampleStreamer.cpp) rather than a bulk NT_readSampleFrames() load into a
// big per-slot buffer - an earlier version used the bulk-load approach and
// it froze the device when all 4 slots loaded a fresh preset at once (4
// simultaneous multi-second reads). Streaming opens near-instantly (see
// MixSampleLayer(), which calls NT_streamOpen() on every trigger to
// restart playback from the top) and pulls only the current block's worth
// of audio each step(), so cost is spread evenly instead of front-loaded.
// `stream` points into DTC, `streamBuffer` into DRAM (both sized from
// NT_globals.streamSizeBytes/streamBufferSizeBytes at construct() time,
// since neither is a compile-time constant).
struct _sampleSlot
{
	int folderIndex;		// -1 = this slot's named folder not found (yet, or at all)
	int numSampleFiles;
	int sampleFileIndex;	// which file within the folder, 0-based; -1 = None selected
	bool hasSample;			// true once a valid (non-None) sample is selected
	float loadedSampleRate;
	uint32_t loadedNumFrames;	// total frames in the file (NT_getSampleFileInfo) - used for idle-tail sizing, not buffering
	// Both split-point candidates - Fixed is a plain formula, computed
	// immediately on selection; Env Follower needs to actually look at the
	// audio, which (now that full-file buffering is gone) means a small,
	// separate, bounded analysis read - see the shared (not per-slot)
	// analysis* fields on _drumMachineAlgorithm and RequestAnalysis().
	// splitFrameEnvFollower defaults to the Fixed value until that
	// analysis completes, so Mix Type is always usable immediately.
	int splitFrameFixed;
	int splitFrameEnvFollower;

	_NT_stream stream;
	void* streamBuffer;

	void Init()
	{
		folderIndex = -1;
		numSampleFiles = 0;
		sampleFileIndex = -1;
		hasSample = false;
		loadedSampleRate = 48000.0f;
		loadedNumFrames = 0;
		splitFrameFixed = 0;
		splitFrameEnvFollower = 0;
	}
};

// Shadow state for one of the 32 smoothed parameters (see kFirstSmoothedParam).
// Normal live edits (from customUi() or host automation/mapping) snap
// `current` straight to the new value - only an explicit preset Load sets
// up a real ramp, over ~1.5 seconds. Read every step() call in place of
// self->v[param] for these parameters.
struct _paramSmoother
{
	float current;
	float target;
	float increment;
	int samplesRemaining;
};

static const int kNumPresets = 64;

// Auto-naming for saved presets: splices one word from each list together
// (e.g. "SHADOW VENOM", "WINTERMOOD BABE") so slots get a recallable name
// instead of a bare number - generated fresh at Save time (see
// GenerateRandomPresetName()/SavePreset()).
static char const * const kPresetNameListA[] = {
	"ARMITAGE", "WINTERMOOD", "RUNNER", "SHADOW", "NEON", "CHROME", "RAZOR",
	"GHOST", "STATIC", "NOVA", "WRAITH", "HAVOC", "VORTEX", "CINDER", "ONYX",
	"ZEPHYR", "VIPER", "CIPHER", "REBEL", "PHANTOM",
};
static char const * const kPresetNameListB[] = {
	"BABE", "LOVER", "FEVER", "BLAZE", "VELVET", "HONEY", "DIESEL",
	"THUNDER", "VENOM", "SILK", "SUGAR", "FOXY", "SLICK", "TURBO", "PRIME",
};
static const int kPresetNameLen = 24;	// longest combo is ~19 chars + null, rounded up

struct _drumMachineAlgorithm : public _NT_algorithm
{
	_drumMachineAlgorithm( _drumMachineAlgorithm_DTC* dtc_ ) : dtc( dtc_ ) {}
	~_drumMachineAlgorithm() {}

	_drumMachineAlgorithm_DTC* dtc;
	float* renderScratch;	// [maxFramesPerStep], points into dram
	float* dryScratch;		// [maxFramesPerStep], points into dram - dry copy for the waveshaper crossfade
	float* elementsExciteScratch;	// [maxFramesPerStep], points into dram - shared, reused sequentially across voices like renderScratch

	// Own, mutable copy of the static `parameters` table - needed so the
	// Sample params' `.max` can be updated per-instance at runtime (once
	// each slot's SD card folder file count is known - see
	// ScanSampleFolders()/NT_updateParameterDefinition()) without one
	// instance's discovered file count leaking into every other loaded
	// instance of this algorithm, which sharing the single static table
	// directly would do. Same pattern as the official distingNT_API
	// samplePlayer.cpp example. Every other read of pThis->parameters[...]
	// elsewhere in this file already goes through this pointer, so no other
	// code needed to change when this stopped pointing at the static table.
	_NT_parameter params[kNumParams];

	_paramSmoother smoother[kNumSmoothedParams];
	bool smoothersInitialized;
	// Non-zero while a preset Load's NT_setParameterFromUi() calls are in
	// flight, so parameterChanged() knows to leave the ramp we just set up
	// alone instead of snapping it (see LoadPreset()).
	int loadingPreset;

	// Custom UI state - the standard page system can't tell draw() which
	// page is selected, so page position and value-editing are handled
	// entirely here instead of via the standard pot/encoder wiring.
	int currentPage;		// one of the kPageXxx values - see kPageType[]/kPageParams[]
	int setupSelectedItem;	// which item is scrolled-to on a list-style setup page
	// List-style setup pages (Routing/MIDI/Mod Matrix) use a 2-stage
	// browse/edit scheme, needed once Mod Matrix has 32 items: rotate
	// Encoder R to scroll while false, push to enter edit mode (true),
	// rotate now adjusts the highlighted item's value, push again to
	// return to browsing. Reset to false on every page change.
	bool setupEditMode;
	// Quick modulation-depth-edit mode on a bar page (button 4): cycles
	// kModSrcNone (Normal) -> kModSrcEnv1 -> kModSrcEnv2 -> kModSrcLfo1 ->
	// kModSrcLfo2 -> Normal - deliberately reuses the kModSrcXxx enum values
	// directly (Normal == kModSrcNone == 0) so this doubles as "which source
	// is being quick-edited". Only meaningful on the 9 concept bar pages
	// (kPageConcept[..] != -1, i.e. not Model). While active, the 4
	// pot/encoder controls edit that source's Mod Matrix depth for each slot
	// instead of the concept's base value - see FindOrAllocRoute()/
	// customUi(). Reset to Normal (0) on every page change.
	int barPageMode;

	// Relative/incremental mode for the 3 bar-page pots (potL/potC/potR),
	// replacing an earlier "wait until the pot crosses the current value"
	// pickup scheme that felt like a dead zone. A pot's *physical* position
	// has nothing to do with whichever page is currently on screen, so
	// instead of mapping absolute position directly, each pot's *movement*
	// since its last reading is added to the parameter's current value -
	// turning it up always moves the value up immediately, no waiting.
	// `potHasLastPos` gates the very first reading after a reset (page
	// change/preset load/UI-open - see customUi()/setupUi()/LoadPreset()) so
	// that first sample only establishes a baseline instead of computing a
	// delta against a stale/zeroed position - see PotRelativeUpdate().
	bool potHasLastPos[3];
	float potLastPos[3];
	// Fractional carry for each pot's accumulated delta (in parameter
	// units, not raw 0..1 pot units) - these are high-resolution pots that
	// report many small position updates per physical turn, so a slow turn
	// produces a stream of tiny deltas that each round to 0 parameter steps
	// on their own. Without this carry, RoundToInt() just throws that
	// movement away every time, so only a fast turn (big delta per update)
	// ever produced a visible change. Now the fraction persists between
	// calls and keeps accumulating until it actually crosses an integer
	// boundary, at which point exactly that much of the parameter moves -
	// same fix shape as _paramSmoother's fractional drift handling. Reset
	// to 0 alongside potHasLastPos (see PotRelativeUpdate()).
	float potAccum[3];

	// Preset menu (button 3 toggles it open/closed; see customUi()). All
	// navigation lives on Encoder R: rotate scrolls the slot list (stage 0),
	// its push button drills into a Load/Save choice for the highlighted
	// slot (stage 1, defaulting to Load so a stray press can never
	// overwrite), rotate again to switch the choice, push again to confirm.
	bool presetMenuOpen;
	int presetMenuStage;	// 0 = browsing slots, 1 = choosing Load/Save
	int presetMenuAction;	// 0 = Load, 1 = Save
	int presetMenuIndex;	// which of kNumPresets slots is selected

	// Delete-modifier confirm dialog - opened by clicking the pot/encoder
	// for a given slot while in quick-edit depth mode on a bar page (see
	// customUi()), same "defaults to the safe choice" pattern as the preset
	// menu's Load/Save. Only ever open while on a bar page in quick-edit
	// mode, so currentPage/barPageMode (read fresh, not snapshotted here)
	// still identify which (source, concept) the dialog is acting on -
	// safe because this dialog owns every claimed control while open, so
	// neither can change out from under it.
	bool deleteConfirmOpen;
	int deleteConfirmSlot;
	int deleteConfirmChoice;	// 0 = No, 1 = Yes

	int16_t presetBank[kNumPresets][kNumKitParams];
	bool presetBankValid[kNumPresets];
	char presetName[kNumPresets][kPresetNameLen];

	// Second preset block (Envelope/LFO globals + Mod Matrix routes - see
	// kModParams/ModParamAt()). Always holds sensible content: construct()
	// seeds every slot with each parameter's own default, SavePreset()
	// overwrites with real values, and deserialise() only touches a slot's
	// entry if its "mod" field is actually present - so a preset saved
	// before this session's changes just keeps the constructed defaults
	// instead of loading garbage.
	int16_t modBank[kNumPresets][kNumModParams];

	// UI-only: last incoming velocity per slot, 0..1, linear-decays back to
	// 0 for a simple peak-meter feel - drawn integrated into each slot's
	// main parameter bar (see DrawBarPage()).
	float velocityMeter[kNumSlots];

	// Mirrors of each voice's current Env1/Env2 level (roughly 0..1, though
	// the SineFold shape corner can go bipolar), updated once per block from
	// step() - lets draw() show the live modulation overlay (see
	// DrawBarPage()) without reaching into DTC.
	float currentEnv1Level[kNumSlots];
	float currentEnv2Level[kNumSlots];

	// MIDI clock (24 PPQN) tracking, driving the synced LFOs - see
	// midiRealtime()/AdvanceLfos(). sampleCounter is a free-running count
	// incremented every step() call; lastClockPulseSample/hasClockPulse let
	// midiRealtime() measure the interval between consecutive clock bytes.
	uint32_t sampleCounter;
	uint32_t lastClockPulseSample;
	bool hasClockPulse;
	float samplesPerQuarterNote;

	// LFO1/LFO2 - both global (not per-voice, unlike the Env1/Env2 envelopes
	// above), each with its own independent rate and shape, advanced once
	// per block in AdvanceLfos(). Values are bipolar (-1..1), read both for
	// modulation (ModulatedViaMatrix()) and for the LFOs page's live display.
	float lfo1Phase, lfo2Phase;
	float lfo1Value, lfo2Value;

	// SD card sample-layer state, one per slot (BD/SD/CH/OH, matching
	// kSlotXxx) - see _sampleSlot's own comment, ScanSampleFolders(), and
	// MixSampleLayer().
	_sampleSlot sampleSlot[kNumSlots];
	// Debounced NT_isSdCardMounted() transition - re-triggers a folder
	// rescan on remount (e.g. a different card with different sample
	// folders), same pattern as the official samplePlayer.cpp example.
	bool sdCardWasMounted;

	// Shared (not per-slot) Env Follower analysis state - a small, bounded
	// NT_readSampleFrames() read (analysisBuffer, ~0.25s, into DRAM - see
	// calculateRequirements()) purely to locate the Knock/Tail split point,
	// entirely separate from actual playback (which streams - see
	// _sampleSlot::stream). Only one slot's analysis is ever in flight at a
	// time - a second/third/fourth request (e.g. all 4 slots getting a
	// fresh sample on one preset load) queues in analysisPending[] instead
	// of firing simultaneously, so even this small read never bursts 4-wide
	// at once. See RequestAnalysis()/AdvanceSampleAnalysis().
	float* analysisBuffer;
	int analysisMaxFrames;
	_NT_wavRequest analysisRequest;
	int analysisCurrentSlot;		// -1 = idle
	bool analysisAwaitingCallback;
	int analysisLoadedNumFrames;
	int analysisScanPos;
	int analysisScanPhase;			// 0 = scanning for the buffer's peak, 1 = scanning post-peak for the decay threshold
	float analysisScanEnv;
	float analysisScanPeak;
	int analysisScanPeakFrame;
	bool analysisPending[kNumSlots];

	// Counts down (in samples, decremented once per step() - see
	// SampleSystemBusy()) after any sample-loading-related activity - a
	// preset load (ours via LoadPreset(), or the host's own via
	// deserialise()), or a live Sample param edit via StartSampleLoad().
	// While non-zero (or while an analysis read is actually in flight -
	// see SampleSystemBusy()), MixSampleLayer() skips opening/rendering
	// its stream entirely, even on trigger. Confirmed by testing: a preset
	// with samples loaded fine with playback stopped, but froze the device
	// loading during active playback - the sample-streaming API's
	// NT_streamOpen()/NT_streamRender() calls (fired on every trigger)
	// colliding with the SD card activity a preset load itself involves
	// (the host's own preset-file read, plus our analysis reads) is the
	// most likely explanation, so this gate keeps triggered hits from
	// touching the sample-streaming API at all until things have settled.
	int sampleSystemGraceSamples;
};

// ---------------------------------------------------------------------
// Requirements / construction
// ---------------------------------------------------------------------

// Per-slot sample buffer capacity - 2 seconds, matching IdleSamples()'s own
// kMaxTailSeconds ceiling (same "as long as any model's decay could ever
// run" reasoning). A separate function (not a constant) since it depends on
// NT_globals.sampleRate, which calculateRequirements() and construct() must
// both derive identically - factored once here so they can never drift
// apart from each other.
//
// Analysis buffer capacity - deliberately small (~0.25s) since it exists
// only to locate the Knock/Tail Env Follower split point, which for a
// drum one-shot is virtually always within the attack/first decay, not
// somewhere deep into a multi-second file. Actual playback no longer
// buffers the file at all (see _sampleSlot::stream) - this is a
// completely separate, much smaller, one-off read.
static int AnalysisMaxFrames()
{
	return (int)( 0.25f * NT_globals.sampleRate );
}

// (Re)starts the sample-system "settle" window - see
// sampleSystemGraceSamples's comment on the algorithm struct. Called from
// anywhere sample-loading-related SD activity might be about to happen:
// LoadPreset(), deserialise(), and StartSampleLoad() (covering both a
// preset load and a live Sample param edit).
static void ExtendSampleSystemGrace( _drumMachineAlgorithm* pThis )
{
	pThis->sampleSystemGraceSamples = (int)( 0.5f * NT_globals.sampleRate );
}

// True while it's not safe to touch the sample-streaming API (see
// MixSampleLayer()) - either an analysis read is actually in flight/queued,
// or we're still within the settle window after some sample-loading-
// related activity (see ExtendSampleSystemGrace()).
static bool SampleSystemBusy( _drumMachineAlgorithm* pThis )
{
	if ( pThis->analysisCurrentSlot != -1 || pThis->sampleSystemGraceSamples > 0 )
		return true;
	for ( int i=0; i<kNumSlots; ++i )
		if ( pThis->analysisPending[i] )
			return true;
	return false;
}

static void StartAnalysisRead( _drumMachineAlgorithm* pThis, int slot );

// Starts the next queued slot's analysis, if any - shared by both the
// normal "scan finished" path and StartAnalysisRead()'s own failure path
// (NT_readSampleFrames() returning false). The failure path used to leave
// any other already-queued slots stuck in analysisPending[] forever, since
// nothing else ever re-checked the queue once analysisCurrentSlot reset to
// -1 outside of a successful scan completion.
static void AdvanceAnalysisQueue( _drumMachineAlgorithm* pThis )
{
	for ( int i=0; i<kNumSlots; ++i )
	{
		if ( pThis->analysisPending[i] )
		{
			StartAnalysisRead( pThis, i );
			return;
		}
	}
}

// Computes the Fixed split point immediately (a plain O(1) formula, always
// available even before/without any analysis read) and queues the Env
// Follower scan (see analysisPending[]) - deliberately never starts the
// actual NT_readSampleFrames() read here, even if nothing else is in
// flight. This can be called from LoadPreset()/deserialise(), which is
// itself already in the middle of an SD card read (the preset file) -
// issuing another SD read synchronously from that same call stack froze
// the device on every preset load that selected a sample, immediately,
// before any trigger, even with just one slot involved (confirmed by
// testing - not proportional to sample count or read size, ruling out the
// earlier bulk-buffer/CPU-overload theories). AdvanceSampleAnalysis(),
// called only from step() on a normal audio-processing cycle - never from
// a parameter-change/preset-load callback - is now the sole place that
// ever actually starts a read, guaranteeing at least one full return to
// the audio loop happens first.
static void RequestAnalysis( _drumMachineAlgorithm* pThis, int slot )
{
	_sampleSlot& s = pThis->sampleSlot[slot];
	constexpr float kFixedKnockSeconds = 0.08f;
	int fixed = (int)( kFixedKnockSeconds * s.loadedSampleRate );
	if ( s.loadedNumFrames > 0 && fixed >= (int)s.loadedNumFrames ) fixed = (int)s.loadedNumFrames - 1;
	if ( fixed < 0 ) fixed = 0;
	s.splitFrameFixed = fixed;
	s.splitFrameEnvFollower = fixed;

	if ( s.loadedNumFrames == 0 )
		return;

	pThis->analysisPending[slot] = true;
}

static void StartAnalysisRead( _drumMachineAlgorithm* pThis, int slot )
{
	_sampleSlot& s = pThis->sampleSlot[slot];
	int numFrames = (int)s.loadedNumFrames;
	if ( numFrames > pThis->analysisMaxFrames ) numFrames = pThis->analysisMaxFrames;

	pThis->analysisCurrentSlot = slot;
	pThis->analysisPending[slot] = false;
	pThis->analysisRequest.folder = s.folderIndex;
	pThis->analysisRequest.sample = s.sampleFileIndex;
	pThis->analysisRequest.dst = pThis->analysisBuffer;
	pThis->analysisRequest.numFrames = numFrames;
	pThis->analysisLoadedNumFrames = numFrames;
	if ( NT_readSampleFrames( pThis->analysisRequest ) )
		pThis->analysisAwaitingCallback = true;
	else
	{
		pThis->analysisCurrentSlot = -1;	// couldn't even start - leave the Fixed fallback, don't get stuck
		AdvanceAnalysisQueue( pThis );		// ...but don't strand any other slots still waiting in the queue
	}
}

// callbackData is `pThis` - there's only ever one analysis request in
// flight for the whole algorithm instance (see analysisCurrentSlot).
static void AnalysisLoadCallback( void* callbackData, bool success )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)callbackData;
	pThis->analysisAwaitingCallback = false;
	if ( !success )
	{
		pThis->analysisCurrentSlot = -1;	// leave the Fixed fallback in place
		return;
	}
	pThis->analysisScanPos = 0;
	pThis->analysisScanPhase = 0;
	pThis->analysisScanEnv = 0.0f;
	pThis->analysisScanPeak = 0.0f;
	pThis->analysisScanPeakFrame = 0;
}

// Advances the current slot's in-flight Env Follower scan by a small,
// fixed chunk of frames - called once per step() (see step()), so the
// full scan (one-pole envelope of |sample|, find peak, then the first
// post-peak frame where it's decayed below 25% of that peak, ~-12dB)
// completes gradually over a handful of blocks instead of costing any one
// block much - even more comfortably now that the analysis buffer itself
// is only ~0.25s (see AnalysisMaxFrames()) rather than up to 2 full
// seconds. Once done, starts the next queued slot's analysis, if any.
static void AdvanceSampleAnalysis( _drumMachineAlgorithm* pThis )
{
	constexpr int kChunkFrames = 2000;
	constexpr float kAttackCoeff = 0.3f;
	constexpr float kReleaseCoeff = 0.01f;

	// The only place a queued analysis read is actually started (see
	// RequestAnalysis()'s comment) - this runs from step(), a normal
	// audio-processing cycle, never synchronously from within a preset
	// load or parameter change.
	if ( pThis->analysisCurrentSlot == -1 )
		AdvanceAnalysisQueue( pThis );

	int slot = pThis->analysisCurrentSlot;
	if ( slot < 0 || pThis->analysisAwaitingCallback )
		return;

	const float* buffer = pThis->analysisBuffer;
	int loadedNumFrames = pThis->analysisLoadedNumFrames;
	bool done = false;

	int remaining = kChunkFrames;
	if ( pThis->analysisScanPhase == 0 )
	{
		while ( remaining > 0 && pThis->analysisScanPos < loadedNumFrames )
		{
			float absIn = fabsf( buffer[pThis->analysisScanPos] );
			float coeff = absIn > pThis->analysisScanEnv ? kAttackCoeff : kReleaseCoeff;
			pThis->analysisScanEnv += ( absIn - pThis->analysisScanEnv ) * coeff;
			if ( pThis->analysisScanEnv > pThis->analysisScanPeak )
			{
				pThis->analysisScanPeak = pThis->analysisScanEnv;
				pThis->analysisScanPeakFrame = pThis->analysisScanPos;
			}
			++pThis->analysisScanPos;
			--remaining;
		}
		if ( pThis->analysisScanPos >= loadedNumFrames )
		{
			if ( pThis->analysisScanPeak <= 0.0f )
				done = true;	// silent buffer - keep the Fixed fallback
			else
			{
				pThis->analysisScanPhase = 1;
				pThis->analysisScanPos = pThis->analysisScanPeakFrame;
				pThis->analysisScanEnv = pThis->analysisScanPeak;
			}
		}
	}
	else
	{
		float threshold = pThis->analysisScanPeak * 0.25f;
		while ( remaining > 0 && pThis->analysisScanPos < loadedNumFrames )
		{
			float absIn = fabsf( buffer[pThis->analysisScanPos] );
			float coeff = absIn > pThis->analysisScanEnv ? kAttackCoeff : kReleaseCoeff;
			pThis->analysisScanEnv += ( absIn - pThis->analysisScanEnv ) * coeff;
			if ( pThis->analysisScanEnv <= threshold ) break;
			++pThis->analysisScanPos;
			--remaining;
		}
		if ( pThis->analysisScanEnv <= threshold || pThis->analysisScanPos >= loadedNumFrames )
		{
			int split = pThis->analysisScanPos;
			if ( split > loadedNumFrames - 1 ) split = loadedNumFrames - 1;
			pThis->sampleSlot[slot].splitFrameEnvFollower = split;
			done = true;
		}
	}

	if ( !done )
		return;

	pThis->analysisCurrentSlot = -1;
	AdvanceAnalysisQueue( pThis );
}

void	calculateRequirements( _NT_algorithmRequirements& req, const int32_t* specifications )
{
	req.numParameters = ARRAY_SIZE(parameters);
	req.sram = sizeof(_drumMachineAlgorithm);
	// Base struct plus kNumSlots' worth of _NT_stream storage tacked on
	// after it - NT_globals.streamSizeBytes is a runtime value (varies by
	// platform build), not a compile-time constant, so it can't be a member
	// of _drumMachineAlgorithm_DTC itself (same reason the official
	// sampleStreamer.cpp example computes its own DTC size this way rather
	// than sizeof()-ing a fixed-size field for it).
	req.dtc = sizeof(_drumMachineAlgorithm_DTC) + kNumSlots * NT_globals.streamSizeBytes;
	// renderScratch + dryScratch + 2x closedHat scratch + 2x openHat scratch
	// + elementsExciteScratch = 7 buffers, each sized to the largest block
	// this instance will ever be asked to render in one step() call; plus
	// one small shared Env Follower analysis buffer (~0.25s - see
	// AnalysisMaxFrames()); plus one streaming buffer per slot (BD/SD/CH/OH)
	// for actual sample playback (NT_streamOpen()/NT_streamRender() - see
	// MixSampleLayer()). No more full-file per-slot buffers - an earlier
	// bulk-load version of this feature froze the device when all 4 slots
	// loaded a fresh preset's samples at once.
	req.dram = 7 * NT_globals.maxFramesPerStep * sizeof(float)
		+ AnalysisMaxFrames() * sizeof(float)
		+ kNumSlots * NT_globals.streamBufferSizeBytes;
	req.itc = 0;
}

_NT_algorithm*	construct( const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req, const int32_t* specifications )
{
	BuildModMatrixPage();	// one-time-equivalent; pure compile-time-derived values, safe to redo per instance

	_drumMachineAlgorithm* alg = new (ptrs.sram) _drumMachineAlgorithm( (_drumMachineAlgorithm_DTC*)ptrs.dtc );

	plaits::kSampleRate = (float)NT_globals.sampleRate;
	elements::kSampleRate = (float)NT_globals.sampleRate;

	float* dram = (float*)ptrs.dram;
	int maxFrames = NT_globals.maxFramesPerStep;
	alg->renderScratch = dram; dram += maxFrames;
	alg->dryScratch = dram; dram += maxFrames;
	alg->dtc->closedHat.scratch1 = dram; dram += maxFrames;
	alg->dtc->closedHat.scratch2 = dram; dram += maxFrames;
	// openHat reuses closedHat's scratch1/scratch2 range is NOT safe (both
	// could be simultaneously "active"), so give it its own - re-derive
	// req.dram sizing note: this needs 4 scratch buffers total, not 3.
	alg->dtc->openHat.scratch1 = dram; dram += maxFrames;
	alg->dtc->openHat.scratch2 = dram; dram += maxFrames;
	alg->elementsExciteScratch = dram; dram += maxFrames;

	alg->analysisMaxFrames = AnalysisMaxFrames();
	alg->analysisBuffer = dram; dram += alg->analysisMaxFrames;

	// Stream buffers (DRAM) continue right after the float-based scratch
	// buffers above, sized in bytes (NT_globals.streamBufferSizeBytes) not
	// floats - hence the cast/byte-pointer arithmetic here. Stream storage
	// itself (opaque _NT_stream state) lives in the DTC bytes tacked on
	// after _drumMachineAlgorithm_DTC's own fixed-size fields - see
	// calculateRequirements()'s matching comment.
	uint8_t* dramBytes = (uint8_t*)dram;
	uint8_t* dtcStreamBase = (uint8_t*)ptrs.dtc + sizeof(_drumMachineAlgorithm_DTC);
	for ( int slot=0; slot<kNumSlots; ++slot )
	{
		_sampleSlot& s = alg->sampleSlot[slot];
		s.Init();
		s.streamBuffer = dramBytes; dramBytes += NT_globals.streamBufferSizeBytes;
		s.stream = (_NT_stream)( dtcStreamBase + slot * NT_globals.streamSizeBytes );
	}

	alg->analysisRequest.callback = AnalysisLoadCallback;
	alg->analysisRequest.callbackData = alg;
	alg->analysisRequest.bits = kNT_WavBits32;
	alg->analysisRequest.channels = kNT_WavMono;
	alg->analysisRequest.progress = kNT_WavNoProgress;
	alg->analysisRequest.startOffset = 0;
	alg->analysisCurrentSlot = -1;
	alg->analysisAwaitingCallback = false;
	for ( int i=0; i<kNumSlots; ++i )
		alg->analysisPending[i] = false;
	alg->sdCardWasMounted = false;
	alg->sampleSystemGraceSamples = 0;

	alg->dtc->kick.Init();
	alg->dtc->snare.Init();
	alg->dtc->closedHat.Init();
	alg->dtc->openHat.Init();

	alg->currentPage = kFirstCustomPage;
	alg->setupSelectedItem = 0;
	alg->setupEditMode = false;
	alg->barPageMode = kModSrcNone;
	alg->potHasLastPos[0] = alg->potHasLastPos[1] = alg->potHasLastPos[2] = false;
	alg->potAccum[0] = alg->potAccum[1] = alg->potAccum[2] = 0.0f;
	alg->smoothersInitialized = false;
	alg->loadingPreset = 0;
	alg->presetMenuOpen = false;
	alg->presetMenuStage = 0;
	alg->presetMenuAction = 0;
	alg->presetMenuIndex = 0;
	alg->deleteConfirmOpen = false;
	alg->deleteConfirmSlot = 0;
	alg->deleteConfirmChoice = 0;
	memset( alg->presetBankValid, 0, sizeof(alg->presetBankValid) );
	memset( alg->velocityMeter, 0, sizeof(alg->velocityMeter) );
	memset( alg->currentEnv1Level, 0, sizeof(alg->currentEnv1Level) );
	memset( alg->currentEnv2Level, 0, sizeof(alg->currentEnv2Level) );
	for ( int i=0; i<kNumPresets; ++i )
		alg->presetName[i][0] = '\0';
	for ( int slot=0; slot<kNumPresets; ++slot )
		for ( int i=0; i<kNumModParams; ++i )
			alg->modBank[slot][i] = (int16_t)parameters[ ModParamAt(i) ].def;

	alg->sampleCounter = 0;
	alg->lastClockPulseSample = 0;
	alg->hasClockPulse = false;
	alg->samplesPerQuarterNote = 0.5f * (float)NT_globals.sampleRate;	// 120bpm fallback until a real clock arrives
	alg->lfo1Phase = 0.0f;
	alg->lfo2Phase = 0.0f;
	alg->lfo1Value = 0.0f;
	alg->lfo2Value = 0.0f;

	memcpy( alg->params, parameters, sizeof(parameters) );
	alg->parameters = alg->params;
	alg->parameterPages = &parameterPages;
	return alg;
}

// Returns the smoothed shadow value for one of the 32 smoothed parameters
// (see kFirstSmoothedParam) - step()'s DSP processing reads this instead
// of self->v[param] directly, so a preset-load fade can ramp the sound
// separately from the (instantly-updated) displayed value.
static float Smoothed( _drumMachineAlgorithm* pThis, int paramIndex )
{
	return pThis->smoother[ paramIndex - kFirstSmoothedParam ].current;
}

// Advances every smoother by one block's worth of samples. Called once per
// step() call, before any voice processing.
static void AdvanceSmoothers( _drumMachineAlgorithm* pThis, int numFrames )
{
	if ( !pThis->smoothersInitialized )
	{
		// First-ever call: seed every smoother directly from the current
		// (default, or deserialised) parameter values, so the very first
		// audio processed matches what's displayed - see the comment on
		// smoothersInitialized in the algorithm struct for why this can't
		// just be done once in construct() instead.
		for ( int i=0; i<kNumSmoothedParams; ++i )
		{
			float v = (float)pThis->v[ kFirstSmoothedParam + i ];
			pThis->smoother[i].current = v;
			pThis->smoother[i].target = v;
			pThis->smoother[i].increment = 0.0f;
			pThis->smoother[i].samplesRemaining = 0;
		}
		pThis->smoothersInitialized = true;
	}

	for ( int i=0; i<kNumSmoothedParams; ++i )
	{
		_paramSmoother& s = pThis->smoother[i];
		if ( s.samplesRemaining <= 0 )
			continue;
		int step = std::min( s.samplesRemaining, numFrames );
		s.current += s.increment * step;
		s.samplesRemaining -= step;
		if ( s.samplesRemaining <= 0 )
			s.current = s.target;	// snap exactly at the end, avoid float drift
	}
}

// Linear decay for the velocity meter bars (UI-only, see the comment on
// velocityMeter on the algorithm struct) - a fixed ~0.4s fall time, no
// transcendental call needed (avoids the libm-symbol-resolution risk an
// expf()-based exponential decay would carry).
static void DecayVelocityMeters( _drumMachineAlgorithm* pThis, int numFrames )
{
	float decay = numFrames / ( 0.4f * plaits::kSampleRate );
	for ( int i=0; i<kNumSlots; ++i )
	{
		pThis->velocityMeter[i] -= decay;
		if ( pThis->velocityMeter[i] < 0.0f ) pThis->velocityMeter[i] = 0.0f;
	}
}

// ---------------------------------------------------------------------
// LFO waveforms - plain piecewise-linear shapes (no libm risk) plus the
// already-vendored plaits::Sine() LUT. `phase` is 0..1, wrapping.
// ---------------------------------------------------------------------

static float Triangle( float phase )
{
	return phase < 0.5f ? ( 4.0f * phase - 1.0f ) : ( 3.0f - 4.0f * phase );
}

static float Saw( float phase )
{
	return 2.0f * phase - 1.0f;
}

static float Square( float phase )
{
	return phase < 0.5f ? 1.0f : -1.0f;
}

// LFO2's one-knob "shape" morph: sine -> triangle -> saw -> square across
// shapeParam 0-100%, the same "traverse corners of a shape space" pattern
// used for the AD/ADSR knobs below - 3 linear crossfade segments across the
// 4 waveforms.
static float MorphedWave( float phase, int shapeParam )
{
	float t = shapeParam * 0.01f;
	if ( t <= 0.3333f )
	{
		float f = t * 3.0f;
		float a = plaits::Sine( phase );
		float b = Triangle( phase );
		return a + f * ( b - a );
	}
	else if ( t <= 0.6667f )
	{
		float f = ( t - 0.3333f ) * 3.0f;
		float a = Triangle( phase );
		float b = Saw( phase );
		return a + f * ( b - a );
	}
	else
	{
		float f = ( t - 0.6667f ) * 3.0f;
		float a = Saw( phase );
		float b = Square( phase );
		return a + f * ( b - a );
	}
}

// Advances one LFO's phase by one block's worth of samples, synced to MIDI
// clock (see midiRealtime()).
static void AdvancePhase( _drumMachineAlgorithm* pThis, float& phase, int rateParam, int numFrames )
{
	int rateIdx = pThis->v[rateParam];
	CONSTRAIN( rateIdx, 0, (int)ARRAY_SIZE(kLfoRateMultiplier) - 1 );
	float periodSamples = pThis->samplesPerQuarterNote * kLfoRateMultiplier[rateIdx];
	if ( periodSamples < 1.0f ) periodSamples = 1.0f;	// guard against div-by-~0 before any clock arrives

	phase += numFrames / periodSamples;
	phase -= (int)phase;
}

// Advances both LFOs by one block's worth of samples - each has its own
// independent rate and shape (previously LFO1 was a fixed-shape/shared-rate
// triangle; both now go through MorphedWave() with their own knobs). Called
// once per step() call, alongside AdvanceSmoothers()/DecayVelocityMeters().
static void AdvanceLfos( _drumMachineAlgorithm* pThis, int numFrames )
{
	AdvancePhase( pThis, pThis->lfo1Phase, kParamLfo1Rate, numFrames );
	AdvancePhase( pThis, pThis->lfo2Phase, kParamLfo2Rate, numFrames );

	pThis->lfo1Value = MorphedWave( pThis->lfo1Phase, pThis->v[kParamLfo1Shape] );
	pThis->lfo2Value = MorphedWave( pThis->lfo2Phase, pThis->v[kParamLfo2Shape] );
}

// ---------------------------------------------------------------------
// AD/ADSR timing mapping (used as the timing basis by 2 of the 4 Envelope
// Shape corners - see BlendEnvelopeLevel()) - pure functions of the Morph
// knob, shared by the actual per-voice runtime envelopes (AdvanceEnvelope())
// and the Envelopes page's live shape preview (DrawEnvelopesPage()), so both
// always agree exactly.
// ---------------------------------------------------------------------

// Converts a normalized 0..1 "how slow" value into an actual time in
// seconds - quadratic taper (matches the filter-cutoff mapping elsewhere in
// this file), avoids needing a pow()/exp() call.
static float NormToSeconds( float norm, float minSeconds, float maxSeconds )
{
	float tt = norm * norm;
	return minSeconds + tt * ( maxSeconds - minSeconds );
}

// AD Shape (0-100%): traverses attack/decay corners in the specified order -
// (fast attack, slow decay) -> (fast attack, fast decay) -> (slow attack,
// fast decay) -> (slow attack, slow decay) - via 3 linear segments.
static void ADShapeToAttackDecay01( int shapeParam, float* attackNorm, float* decayNorm )
{
	float t = shapeParam * 0.01f;
	if ( t <= 0.3333f )
	{
		*attackNorm = 0.0f;
		*decayNorm = 1.0f - t * 3.0f;
	}
	else if ( t <= 0.6667f )
	{
		*attackNorm = ( t - 0.3333f ) * 3.0f;
		*decayNorm = 0.0f;
	}
	else
	{
		*attackNorm = 1.0f;
		*decayNorm = ( t - 0.6667f ) * 3.0f;
	}
}

static void ADShapeToTimes( int shapeParam, float* attackS, float* decayS )
{
	float attackNorm, decayNorm;
	ADShapeToAttackDecay01( shapeParam, &attackNorm, &decayNorm );
	*attackS = NormToSeconds( attackNorm, 0.002f, 0.3f );
	*decayS = NormToSeconds( decayNorm, 0.02f, 2.0f );
}

// ADSR Shape (0-100%): every trigger here is a one-shot MIDI note-on with no
// note-off handling, so a gate-driven Sustain/Release doesn't apply -
// implemented as a fixed-duration AHDSR instead (Attack -> Decay to a
// sustain level -> Hold that level for a bit -> Release to 0), all
// independently time-based. The one knob scales every segment's duration
// (short/percussive at 0%, long pad-like swell at 100%) plus the sustain
// level together.
static void ADSRShapeToTimes( int shapeParam, float* attackS, float* decayS, float* holdS, float* releaseS, float* sustainLevel )
{
	float t = shapeParam * 0.01f;
	*attackS = NormToSeconds( t, 0.005f, 0.3f );
	*decayS = NormToSeconds( t, 0.02f, 0.5f );
	*holdS = NormToSeconds( t, 0.02f, 0.8f );
	*releaseS = NormToSeconds( t, 0.05f, 1.5f );
	*sustainLevel = 0.3f + 0.5f * t;
}

// Envelope Shape (0-100%): each of Env1/Env2 morphs its *contour family*
// through 4 corners - AD -> ADSR -> Random (S&H-style decaying stepped
// contour) -> Sine-fold (decaying folded-sine contour, reusing the already-
// vendored plaits::Sine() LUT) - via the same 3-segment crossfade pattern as
// MorphedWave()'s LFO shape morph. Each corner function is a pure function of
// elapsed time since trigger (no internal stage machine), so two neighboring
// corners can be evaluated and blended at any instant regardless of how far
// into the envelope we are - see BlendEnvelopeLevel(). All 4 corners reuse
// Morph's attack/decay (or ADSR) timing as their duration basis, so Morph
// keeps controlling overall pace/balance no matter which Shape corner (or
// blend) is selected.

static float LevelAD( float elapsed, int morphParam, bool* finished )
{
	float attackS, decayS;
	ADShapeToTimes( morphParam, &attackS, &decayS );
	if ( elapsed < attackS )
	{
		*finished = false;
		return elapsed / attackS;
	}
	float t = elapsed - attackS;
	if ( t >= decayS )
	{
		*finished = true;
		return 0.0f;
	}
	*finished = false;
	return 1.0f - t / decayS;
}

static float LevelADSR( float elapsed, int morphParam, bool* finished )
{
	float attackS, decayS, holdS, releaseS, sustainLevel;
	ADSRShapeToTimes( morphParam, &attackS, &decayS, &holdS, &releaseS, &sustainLevel );
	if ( elapsed < attackS )
	{
		*finished = false;
		return elapsed / attackS;
	}
	float t = elapsed - attackS;
	if ( t < decayS )
	{
		*finished = false;
		return 1.0f - ( 1.0f - sustainLevel ) * ( t / decayS );
	}
	t -= decayS;
	if ( t < holdS )
	{
		*finished = false;
		return sustainLevel;
	}
	t -= holdS;
	if ( t >= releaseS )
	{
		*finished = true;
		return 0.0f;
	}
	*finished = false;
	return sustainLevel * ( 1.0f - t / releaseS );
}

// Decaying stepped (sample & hold) contour - `steps` holds this trigger's
// randomly-generated levels (see AdvanceEnvelope()), held for equal slices of
// the AD family's total (attack+decay) duration and faded toward 0 overall.
static float LevelRandom( float elapsed, int morphParam, const float* steps, int numSteps, bool* finished )
{
	float attackS, decayS;
	ADShapeToTimes( morphParam, &attackS, &decayS );
	float total = attackS + decayS;
	if ( elapsed >= total )
	{
		*finished = true;
		return 0.0f;
	}
	*finished = false;
	float frac = elapsed / total;
	int stepIdx = (int)( frac * numSteps );
	CONSTRAIN( stepIdx, 0, numSteps - 1 );
	return steps[stepIdx] * ( 1.0f - frac );
}

// Decaying folded-sine contour - similar in spirit to a Zadar-style "more
// exotic" envelope waveshape. Bipolar (unlike the other 3 corners), which is
// fine for a modulation source (same as the LFOs).
static float LevelSineFold( float elapsed, int morphParam, bool* finished )
{
	float attackS, decayS;
	ADShapeToTimes( morphParam, &attackS, &decayS );
	float total = attackS + decayS;
	if ( elapsed >= total )
	{
		*finished = true;
		return 0.0f;
	}
	*finished = false;
	float frac = elapsed / total;
	float ph = frac * 3.0f;
	ph -= (int)ph;
	float folded = HardClip( 2.5f * plaits::Sine( ph ) );
	return folded * ( 1.0f - frac );
}

// 3-segment crossfade across the 4 corners (AD -> ADSR -> Random -> SineFold),
// same pattern as MorphedWave(). Evaluates the 2 neighboring corner functions
// every call (cheap - arithmetic plus at most one Sine() LUT lookup) and
// linearly blends both level and "finished" state; only truly finished once
// *both* neighbors report finished, so morphing near a corner boundary never
// cuts the envelope off early.
static float BlendEnvelopeLevel( float elapsed, int morphParam, int shapeParam, const float* randomSteps, bool* finished )
{
	float t = shapeParam * 0.01f;
	bool fA, fB;
	float a, b, f;
	if ( t <= 0.3333f )
	{
		f = t * 3.0f;
		a = LevelAD( elapsed, morphParam, &fA );
		b = LevelADSR( elapsed, morphParam, &fB );
	}
	else if ( t <= 0.6667f )
	{
		f = ( t - 0.3333f ) * 3.0f;
		a = LevelADSR( elapsed, morphParam, &fA );
		b = LevelRandom( elapsed, morphParam, randomSteps, 6, &fB );
	}
	else
	{
		f = ( t - 0.6667f ) * 3.0f;
		a = LevelRandom( elapsed, morphParam, randomSteps, 6, &fA );
		b = LevelSineFold( elapsed, morphParam, &fB );
	}
	*finished = fA && fB;
	return a + f * ( b - a );
}

// Advances one voice's envelope by one block; returns the current level
// (already accent-scaled - roughly 0..1, though the SineFold shape corner
// can go bipolar). `blockSeconds` = numFrames/sampleRate.
static float AdvanceEnvelope( _envelopeState& env, bool trig, float accent, int morphParam, int shapeParam, float blockSeconds )
{
	if ( trig )
	{
		env.active = true;
		env.elapsed = 0.0f;
		env.peak = 0.5f + 0.5f * accent;
		for ( int i=0; i<6; ++i )
			env.randomSteps[i] = stmlib::Random::GetFloat();
	}
	if ( !env.active )
		return 0.0f;

	env.elapsed += blockSeconds;
	bool finished;
	float level = BlendEnvelopeLevel( env.elapsed, morphParam, shapeParam, env.randomSteps, &finished );
	if ( finished )
	{
		env.active = false;
		return 0.0f;
	}
	return env.peak * level;
}

// ---------------------------------------------------------------------
// Mod Matrix - loops the 8 assignable routes, applying any whose Target Slot/
// Concept match this call, using a cheap integer compare to skip everything
// else (None-source routes and non-matching routes cost one compare each,
// not a multiply-add) - see kNumModRoutes's comment for why this replaced an
// earlier dense per-concept-per-slot-per-source parameter grid.
// ---------------------------------------------------------------------

// Depth is a *percentage of the concept's own range* (-100..100, same
// convention the bar-page overlay already displays it in - see
// DrawBarPage()'s per-source thin lines, which scale by the live
// parameter's own p.max-p.min): 100% depth means a source at full level can
// swing the value across its entire range. All 4 slots of a given concept
// share one min/max in `parameters[]`, so a flat per-concept table is exact,
// not an approximation. (Bug fix: this used to be missing entirely - depth
// was applied as a flat +-1.0-unit swing regardless of the concept's actual
// range, e.g. +-1 out of Pitch's 0..127, which was never audible.)
static const float kConceptRange[kNumConcepts] = {
	100.0f,	// Release
	100.0f,	// Compressor
	200.0f,	// Filter (-100..100)
	100.0f,	// Waveshaper/Drive
	127.0f,	// Pitch
	200.0f,	// Volume (0..200)
	100.0f,	// Tone
	100.0f,	// Character
	100.0f,	// FM
	100.0f,	// Wavefolder
};

static float ModulatedViaMatrix( _drumMachineAlgorithm* pThis, int concept, int slot, float baseValue, float env1Level, float env2Level )
{
	float result = baseValue;
	for ( int r=0; r<kNumModRoutes; ++r )
	{
		int source = pThis->v[ ModRouteParam( r, kModParamSource ) ];
		if ( source == kModSrcNone )
			continue;
		if ( pThis->v[ ModRouteParam( r, kModParamSlot ) ] != slot )
			continue;
		if ( pThis->v[ ModRouteParam( r, kModParamConcept ) ] != concept )
			continue;

		float srcValue;
		switch ( source )
		{
		case kModSrcEnv1: srcValue = env1Level; break;
		case kModSrcEnv2: srcValue = env2Level; break;
		case kModSrcLfo1: srcValue = pThis->lfo1Value; break;
		default:          srcValue = pThis->lfo2Value; break;	// kModSrcLfo2
		}
		float depth = (float)pThis->v[ ModRouteParam( r, kModParamDepth ) ] * 0.01f;
		result += srcValue * depth * kConceptRange[concept];
	}
	return result;
}

// True if at least one of the kNumModRoutes routes is still unclaimed
// (Source == None) - used to show a "FULL" indicator on the bar page
// instead of silently doing nothing when quick-edit (button 4) can't find
// or allocate a route for an unmapped slot (see DrawBarPage()).
static bool AnyRouteFree( _drumMachineAlgorithm* pThis )
{
	for ( int r=0; r<kNumModRoutes; ++r )
		if ( pThis->v[ ModRouteParam( r, kModParamSource ) ] == kModSrcNone )
			return true;
	return false;
}

// Read-only lookup: the route currently mapping `source` to (concept, slot),
// or -1 if none - used for display (DrawBarPage()) and as the first step of
// FindOrAllocRoute() below. Never mutates, unlike FindOrAllocRoute().
static int FindRoute( _drumMachineAlgorithm* pThis, int source, int slot, int concept )
{
	for ( int r=0; r<kNumModRoutes; ++r )
	{
		if ( pThis->v[ ModRouteParam( r, kModParamSource ) ] != source )
			continue;
		if ( pThis->v[ ModRouteParam( r, kModParamSlot ) ] != slot )
			continue;
		if ( pThis->v[ ModRouteParam( r, kModParamConcept ) ] != concept )
			continue;
		return r;
	}
	return -1;
}

// Coarser, per-BLOCK approximation of the Envelope/Feedback FM family (see
// kFmModeXxx's comment) for models with no internal per-sample FM hook of
// their own (Elements, plaits::HiHat, braids::Cymbal) - Analog/Synthetic/808
// compute this same family sample-by-sample inside their own Render() calls
// instead, since they already had a natural place to hook in; these models
// don't, so a single push value is computed once per block here and applied
// to f0/pitch before calling into them. Returns a small multiplier-ish term
// (order -0.9..3.0, same safety-clamped range used internally by the other
// models) - the caller scales it by its own FM amount and a model-
// appropriate gain.
static float ComputeExternalFmPush( _drumVoicePost& v, bool trig, float accent, int fmMode, float blockSeconds )
{
	// Decays by *elapsed time*, not by call count - a per-block multiply
	// (the first version of this) decays at wildly different real-world
	// rates depending on how many samples happen to be in a block, and at
	// this platform's actual block sizes it collapsed to ~0 within a
	// fraction of a millisecond, well before it could ever be audible (see
	// the "not really audible on the hihat models" bug report). A plain
	// linear decay over a fixed ~30ms window - matching the same rough
	// timescale as the other models' own envelope-driven knock - avoids
	// needing exp()/expf() (see distingnt_libm_symbol_constraints project
	// memory) while staying independent of block size.
	const float kEnvTotalS = 0.03f;
	if ( trig )
	{
		v.fmEnvLevel = 1.0f;
		v.fmEnvElapsed = 0.0f;
		if ( fmMode >= 1 )
			v.fmPrevSample = accent * 1.5f;
	}
	else
	{
		v.fmEnvElapsed += blockSeconds;
		v.fmEnvLevel = 1.0f - v.fmEnvElapsed / kEnvTotalS;
		if ( v.fmEnvLevel < 0.0f ) v.fmEnvLevel = 0.0f;
	}

	if ( fmMode == 0 )
		return v.fmEnvLevel;

	const float kRange = 1.5f;
	float fb = v.fmPrevSample;
	CONSTRAIN( fb, -kRange, kRange );
	float mag = fabsf( fb ) * ( 1.0f / kRange );
	float sign = fb < 0.0f ? -1.0f : 1.0f;
	float shaped = mag;
	if ( fmMode == 2 ) shaped = mag * mag;
	else if ( fmMode == 3 ) shaped = mag * mag * mag;
	else if ( fmMode == 4 ) shaped = mag * ( 2.0f - mag );
	float feedbackTerm = sign * shaped;

	float total = ( fmMode == 5 ) ? ( v.fmEnvLevel + feedbackTerm ) : feedbackTerm;
	CONSTRAIN( total, -0.9f, 3.0f );
	return total;
}

// Captures this block's last rendered sample for the next block's feedback -
// call once after any Render() that used ComputeExternalFmPush()'s result.
static void UpdateFmFeedback( _drumVoicePost& v, const float* scratch, int numFrames )
{
	if ( numFrames > 0 )
		v.fmPrevSample = scratch[numFrames - 1];
}

// Shared "Elements" model glue - a struck-mallet excitation into a modal
// resonator bank (see third_party/mi_elements/ATTRIBUTION.md). Reuses this
// voice's existing knobs rather than exposing new ones: Pitch->frequency,
// Release->damping (higher = longer ring, matching every other voice's
// decay knob - Elements' own "damping" convention is inverted from what
// the name suggests), Tone->brightness+exciter timbre, Character->
// geometry+exciter parameter (each pair shares one knob since Elements
// wants 6 distinct controls and this project budgets 4 per voice).
static void RenderElements( elements::Exciter& exciter, elements::Resonator& resonator, float* exciteScratch, bool trig, float accent, float f0, float tone, float decay, float character, float* out, int numFrames )
{
	exciter.set_timbre( tone );
	exciter.set_parameter( character );
	resonator.set_frequency( f0 );
	resonator.set_brightness( tone );
	resonator.set_damping( decay );
	resonator.set_geometry( character );

	exciter.Process( trig, exciteScratch, numFrames );
	resonator.Process( exciteScratch, out, numFrames );

	for ( int i=0; i<numFrames; ++i )
		out[i] *= accent;
}

// Defined later (customUi() section) - forward-declared so the sample-layer
// helpers below (used by parameterChanged(), which comes first) can clamp a
// Sample param's value the same way any other live edit does.
static void SetParam( _drumMachineAlgorithm* pThis, int paramIndex, int value );

static const int kSampleParam[kNumSlots] = { kParamSampleBD, kParamSampleSD, kParamSampleCH, kParamSampleOH };

// Finds this slot's SD card sample folder by exact name match against
// kSlotNames ("BD"/"SD"/"CH"/"OH") and records its file count - called once
// per mount (see step()'s NT_isSdCardMounted() transition check). A slot
// whose folder doesn't exist (no such subfolder, or no card at all) just
// keeps folderIndex at -1 and its Sample param stuck at max=0/"None" -
// matches "no sample by default" needing no extra guard anywhere else.
static void ScanSampleFolders( _drumMachineAlgorithm* pThis )
{
	int algIdx = NT_algorithmIndex( pThis );
	int numFolders = (int)NT_getNumSampleFolders();
	for ( int slot=0; slot<kNumSlots; ++slot )
	{
		_sampleSlot& s = pThis->sampleSlot[slot];
		s.folderIndex = -1;
		s.numSampleFiles = 0;
		for ( int f=0; f<numFolders; ++f )
		{
			_NT_wavFolderInfo info;
			NT_getSampleFolderInfo( f, info );
			if ( info.name && strcmp( info.name, kSlotNames[slot] ) == 0 )
			{
				s.folderIndex = f;
				s.numSampleFiles = (int)info.numSampleFiles;
				break;
			}
		}
		int paramIndex = kSampleParam[slot];
		pThis->params[paramIndex].max = s.numSampleFiles;
		NT_updateParameterDefinition( algIdx, paramIndex );
		if ( pThis->v[paramIndex] > s.numSampleFiles )
			SetParam( pThis, paramIndex, s.numSampleFiles );
	}
}

// Responds to a slot's Sample param changing (see parameterChanged()) -
// looks up the file's metadata (sample rate/frame count, needed for
// playback speed and idle-tail sizing) and queues the Knock/Tail Env
// Follower analysis (see RequestAnalysis()). Deliberately does NOT open
// the actual playback stream here - that happens per-trigger instead (see
// OpenSampleStream(), called from MixSampleLayer()), since a persistent
// "kit" choice (which sample is loaded into this slot) and "start playing
// it now" are different events for a drum machine that retriggers the same
// sample on every hit, unlike a one-shot demo that streams once from
// selection onward.
static void StartSampleLoad( _drumMachineAlgorithm* pThis, int slot )
{
	_sampleSlot& s = pThis->sampleSlot[slot];
	int value = pThis->v[ kSampleParam[slot] ];
	if ( value <= 0 || s.folderIndex < 0 )
	{
		s.hasSample = false;
		s.sampleFileIndex = -1;
		return;
	}

	int fileIndex = value - 1;
	// LoadPreset() calls this unconditionally for all 4 slots on every
	// preset load, even when a slot's sample selection hasn't actually
	// changed - skip the metadata lookup and (more importantly) queuing
	// another analysis read entirely in that case, rather than needlessly
	// re-reading and re-scanning a sample that's already loaded and
	// already has a valid, up-to-date splitFrameEnvFollower.
	if ( s.hasSample && s.sampleFileIndex == fileIndex )
		return;

	_NT_wavInfo info;
	NT_getSampleFileInfo( s.folderIndex, fileIndex, info );
	s.loadedSampleRate = info.sampleRate > 0 ? (float)info.sampleRate : 48000.0f;
	s.loadedNumFrames = info.numFrames;
	s.sampleFileIndex = fileIndex;
	s.hasSample = true;

	ExtendSampleSystemGrace( pThis );
	RequestAnalysis( pThis, slot );
}

// Called at trigger time (see MixSampleLayer()) - (re)starts this slot's
// stream from the top of the currently-selected file. NT_streamOpen() is a
// lightweight seek/reset, not a bulk read (see distingNT_API/examples/
// sampleStreamer.cpp), so calling it on every hit (rather than once at
// selection time) is the intended usage for one-shot retriggering.
static void OpenSampleStream( _drumMachineAlgorithm* pThis, int slot, float accent )
{
	_sampleSlot& s = pThis->sampleSlot[slot];
	_NT_streamOpenData data = {
		.streamBuffer = s.streamBuffer,
		.folder = (uint32_t)s.folderIndex,
		.sample = (uint32_t)s.sampleFileIndex,
		.velocity = accent,
		.startOffset = 0,
		.reverse = false,
		.rrMode = kNT_RRModeSequential,
	};
	NT_streamOpen( s.stream, data );
}

// Shared by parameterString() (standard menu) and DrawBarPage() (custom UI)
// - value 0 = "None" (the default/no-sample state), else looks up the
// (value-1)'th file's name in this slot's own folder. Returns false (no
// name available - e.g. folder not found yet) if nothing was written.
static bool GetSampleDisplayName( _drumMachineAlgorithm* pThis, int slot, int value, char* buff, int buffSize )
{
	if ( value <= 0 )
	{
		strncpy( buff, "None", buffSize - 1 );
		buff[buffSize-1] = 0;
		return true;
	}
	_sampleSlot& s = pThis->sampleSlot[slot];
	if ( s.folderIndex < 0 )
		return false;
	_NT_wavInfo info;
	NT_getSampleFileInfo( s.folderIndex, value - 1, info );
	if ( !info.name )
		return false;
	strncpy( buff, info.name, buffSize - 1 );
	buff[buffSize-1] = 0;
	return true;
}

int	parameterString( _NT_algorithm* self, int p, int v, char* buff )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;
	int slot;
	if ( p == kParamSampleBD ) slot = kSlotBD;
	else if ( p == kParamSampleSD ) slot = kSlotSD;
	else if ( p == kParamSampleCH ) slot = kSlotCH;
	else if ( p == kParamSampleOH ) slot = kSlotOH;
	else return 0;

	if ( !GetSampleDisplayName( pThis, slot, v, buff, kNT_parameterStringSize ) )
		return 0;
	return (int)strlen( buff );
}

void	parameterChanged( _NT_algorithm* self, int p )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;
	if ( p == kParamSampleBD ) StartSampleLoad( pThis, kSlotBD );
	else if ( p == kParamSampleSD ) StartSampleLoad( pThis, kSlotSD );
	else if ( p == kParamSampleCH ) StartSampleLoad( pThis, kSlotCH );
	else if ( p == kParamSampleOH ) StartSampleLoad( pThis, kSlotOH );
	else if ( p == kParamMidiMode )
	{
		bool notePerSlot = ( pThis->v[kParamMidiMode] == 0 );
		int algIdx = NT_algorithmIndex( self );
		uint32_t off = NT_parameterOffset();
		NT_setParameterGrayedOut( algIdx, kParamMidiChannel + off, !notePerSlot );
		NT_setParameterGrayedOut( algIdx, kParamMidiNoteBD + off, !notePerSlot );
		NT_setParameterGrayedOut( algIdx, kParamMidiNoteSD + off, !notePerSlot );
		NT_setParameterGrayedOut( algIdx, kParamMidiNoteCH + off, !notePerSlot );
		NT_setParameterGrayedOut( algIdx, kParamMidiNoteOH + off, !notePerSlot );
		NT_setParameterGrayedOut( algIdx, kParamMidiChBD + off, notePerSlot );
		NT_setParameterGrayedOut( algIdx, kParamMidiChSD + off, notePerSlot );
		NT_setParameterGrayedOut( algIdx, kParamMidiChCH + off, notePerSlot );
		NT_setParameterGrayedOut( algIdx, kParamMidiChOH + off, notePerSlot );
	}
	else if ( p >= kFirstSmoothedParam && p < kFirstSmoothedParam + kNumSmoothedParams && !pThis->loadingPreset )
	{
		// Live edit (customUi(), or host automation/mapping bypassing our
		// UI entirely) - snap instantly, no fade. Only an explicit preset
		// Load (see LoadPreset()) sets up a real ramp, and it leaves this
		// alone via the loadingPreset guard above.
		_paramSmoother& s = pThis->smoother[ p - kFirstSmoothedParam ];
		s.current = s.target = (float)pThis->v[p];
		s.increment = 0.0f;
		s.samplesRemaining = 0;
	}
}

// ---------------------------------------------------------------------
// MIDI triggering
// ---------------------------------------------------------------------

static void TriggerSlot( _drumMachineAlgorithm* pThis, int slot, float accent )
{
	switch ( slot )
	{
	case kSlotBD: pThis->dtc->kick.pendingTrigger = true; pThis->dtc->kick.pendingAccent = accent; break;
	case kSlotSD: pThis->dtc->snare.pendingTrigger = true; pThis->dtc->snare.pendingAccent = accent; break;
	case kSlotCH: pThis->dtc->closedHat.pendingTrigger = true; pThis->dtc->closedHat.pendingAccent = accent; break;
	case kSlotOH: pThis->dtc->openHat.pendingTrigger = true; pThis->dtc->openHat.pendingAccent = accent; break;
	}
	pThis->velocityMeter[slot] = accent;
}

void	midiMessage( _NT_algorithm* self, uint8_t byte0, uint8_t byte1, uint8_t byte2 )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;

	if ( ( byte0 & 0xF0 ) != 0x90 || byte2 == 0 )
		return;	// only note-on with velocity > 0 triggers; note-off is ignored

	int channel = ( byte0 & 0x0F ) + 1;	// 1-16
	int note = byte1;
	float accent = byte2 * ( 1.0f / 127.0f );

	bool notePerSlot = ( pThis->v[kParamMidiMode] == 0 );

	static const int kNoteParam[kNumSlots] = { kParamMidiNoteBD, kParamMidiNoteSD, kParamMidiNoteCH, kParamMidiNoteOH };
	static const int kChannelParam[kNumSlots] = { kParamMidiChBD, kParamMidiChSD, kParamMidiChCH, kParamMidiChOH };

	for ( int slot = 0; slot < kNumSlots; ++slot )
	{
		bool match;
		if ( notePerSlot )
		{
			int listenChannel = pThis->v[kParamMidiChannel];
			bool channelOk = ( listenChannel == 0 ) || ( listenChannel == channel );
			match = channelOk && ( note == pThis->v[ kNoteParam[slot] ] );
		}
		else
		{
			match = ( channel == pThis->v[ kChannelParam[slot] ] );
		}
		if ( match )
			TriggerSlot( pThis, slot, accent );
	}
}

// Realtime MIDI messages (clock/start/stop) - a separate callback from
// midiMessage()'s note handling. Drives the synced LFOs (see AdvanceLfos()).
void	midiRealtime( _NT_algorithm* self, uint8_t byte )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;

	if ( byte == 0xF8 )	// clock, 24 pulses per quarter note
	{
		if ( pThis->hasClockPulse )
		{
			uint32_t samplesPerPulse = pThis->sampleCounter - pThis->lastClockPulseSample;
			float newEstimate = (float)samplesPerPulse * 24.0f;
			// Light smoothing - one incoming clock's jitter shouldn't visibly
			// wobble the LFOs.
			pThis->samplesPerQuarterNote += 0.25f * ( newEstimate - pThis->samplesPerQuarterNote );
		}
		pThis->lastClockPulseSample = pThis->sampleCounter;
		pThis->hasClockPulse = true;
	}
	else if ( byte == 0xFA || byte == 0xFB )	// start / continue
	{
		pThis->lfo1Phase = 0.0f;
		pThis->lfo2Phase = 0.0f;
	}
}

// ---------------------------------------------------------------------
// Audio rendering
// ---------------------------------------------------------------------

// Idle tail scaled to the slot's own release/decay setting - a short, punchy
// hit (most common case) no longer pays for a full 2s of active rendering
// after it's inaudible, which was the single biggest CPU cost under real
// playback (every voice re-triggered before its fixed tail expired means
// the idle-skip optimisation barely engages at all). `decay01` is the same
// 0..1 decay/release value already read for the voice's Render() call.
static int IdleSamples( float decay01 )
{
	constexpr float kMinTailSeconds = 0.15f;
	constexpr float kMaxTailSeconds = 2.0f;
	float seconds = kMinTailSeconds + decay01 * ( kMaxTailSeconds - kMinTailSeconds );
	return (int)( plaits::kSampleRate * seconds );
}

// Volume page gives up to +6dB of headroom (max raised 100->200%) for slots
// that need to go louder - applied only when the user actually turns that
// knob up, not as an always-on multiplier (a flat always-on boost here
// previously pushed the signal into unguarded clipping at default settings,
// audibly changing the drum character - reverted).

// Fast feed-forward peak compressor redesigned for punch, not just gain
// reduction - the original had a very slow release (kReleaseCoeff=0.001,
// ~1000-sample time constant), so gain reduction from one hit was still
// recovering when the next hit landed, blunting its attack and reading as
// *less* punchy the harder it worked; it also had no makeup gain, so heavier
// settings were just quieter. Both coefficients are now fast (short attack,
// short release - release un-ducks well before the next typical hit), and
// automatic makeup gain keeps the average level up so only true peaks get
// limited. Runs at the *end* of the chain now (see ApplyPost()), after
// Filter/Waveshaper/Volume, like a bus compressor rather than a pre-tone-
// shaping stage. The per-slot knob only controls threshold (100% = low
// threshold = heavy compression, bypassed entirely at 0 like Filter/Drive's
// "0 = no-op" convention).
static void ApplyCompressor( _drumVoicePost& v, float* buf, int numFrames, int compParam )
{
	if ( compParam <= 0 )
		return;

	constexpr float kAttackCoeff = 0.35f;
	constexpr float kReleaseCoeff = 0.08f;
	constexpr float kRatio = 6.0f;
	float threshold = 1.0f - ( compParam * 0.01f ) * 0.9f;	// 100%->0.1 (heavy), ~0%->1.0 (light)
	float makeup = 1.0f / threshold;	// compensate so heavier settings don't just get quieter

	float env = v.compEnvelope;
	for ( int i=0; i<numFrames; ++i )
	{
		float absIn = buf[i] < 0.0f ? -buf[i] : buf[i];
		float coeff = absIn > env ? kAttackCoeff : kReleaseCoeff;
		ONE_POLE( env, absIn, coeff );

		float gain = 1.0f;
		if ( env > threshold )
		{
			float over = env - threshold;
			gain = ( threshold + over / kRatio ) / env;
		}
		buf[i] *= gain * makeup;
	}
	v.compEnvelope = env;
}

// A fixed pre-gain into a hard clip - deliberately cheap/bounded and not
// another vendored MI file; this is "model 2", the harsher waveshaper the
// 50-100% range crossfades into (see ApplyPost()).
static float HardClip( float x )
{
	x *= 4.0f;
	CONSTRAIN( x, -1.0f, 1.0f );
	return x;
}

// DJ-style bipolar filter (center = bypass, down = lowpass darkening, up =
// highpass thinning) -> 2-stage waveshaper -> output level trim -> compressor
// (last, bus-style - see ApplyCompressor()), applied in place. `dryBuf` is
// scratch space (same size as `buf`) used to hold an unprocessed copy for
// the waveshaper's dry/wet crossfade.
// Reflects x into [-1,1] via a bounded number of fold-point reflections -
// purely arithmetic (branch/subtract only, no libm), safe since `x` is a
// bounded audio sample scaled by a capped drive amount, so the number of
// reflections needed to settle is always small in practice; the fixed
// iteration count below is just a safety ceiling against a pathological
// input, not the normal-case cost.
static float ReflectFold( float x )
{
	for ( int iter = 0; iter < 8; ++iter )
	{
		if ( x > 1.0f ) x = 2.0f - x;
		else if ( x < -1.0f ) x = -2.0f - x;
		else break;
	}
	return x;
}

// Post-fx wavefolder - runs between Filter and Waveshaper in ApplyPost()
// (folding generates new harmonics right after tone-shaping, then the
// Waveshaper's soft/hard clip acts on the already-folded signal - the
// alternative, folding an already-clipped signal, tends to sound chaotic
// rather than characterful). `amount` is 0..1. All three types are
// libm-safe (LUT or rational-polynomial only - no tanh/sinh/atan, which
// the firmware's plugin loader isn't confirmed to resolve, see
// distingnt_libm_symbol_constraints project memory) rather than literal
// ports of any one specific reference circuit (Buchla 259 etc. reference
// models are typically tanh/diode-table based).
static void ApplyWavefolder( float* buf, int numFrames, float amount, int type )
{
	if ( amount <= 0.0f )
		return;

	float drive = 1.0f + amount * 7.0f;	// 0%->1x (near passthrough), 100%->8x (deep multi-fold)

	if ( type == kWavefolderTypeSine )
	{
		// Classic West Coast sine fold - reuses the already-vendored,
		// LUT-based plaits::Sine() (safe for phase >= 0.0f, wraps
		// internally - see sine_oscillator.h). Magnitude/sign split keeps
		// the phase argument always non-negative while preserving the
		// fold's odd symmetry.
		for ( int i=0; i<numFrames; ++i )
		{
			float x = buf[i];
			float mag = fabsf( x ) * drive;
			float sign = x < 0.0f ? -1.0f : 1.0f;
			buf[i] = sign * plaits::Sine( mag * 0.5f );
		}
	}
	else if ( type == kWavefolderTypeTriangle )
	{
		// Serge-style hard reflect fold - harder-edged/more digital than
		// Sine, with audible sharp corners at each fold point.
		for ( int i=0; i<numFrames; ++i )
			buf[i] = ReflectFold( buf[i] * drive );
	}
	else
	{
		// Buchla-style: the same reflect fold, then rounded off with a
		// cheap rational ease-out curve (same family as Overdrive's
		// soft-clip below) to soften the reflection's sharp corners for a
		// smoother, more "analog" character than the raw Triangle fold.
		for ( int i=0; i<numFrames; ++i )
		{
			float folded = ReflectFold( buf[i] * drive );
			float mag = fabsf( folded );
			float sign = folded < 0.0f ? -1.0f : 1.0f;
			buf[i] = sign * mag * ( 1.5f - 0.5f * mag * mag );
		}
	}
}

static void ApplyPost( _drumVoicePost& v, float* buf, float* dryBuf, int numFrames, int filterParam, int foldParam, int foldType, int compParam, int driveParam, int volParam )
{
	if ( filterParam < 0 )
	{
		if ( filterParam != v.cachedFilterParam )
		{
			float t = ( -filterParam ) * 0.01f;
			float tt = t * t;	// quadratic taper - avoids needing a pow() call
			float cutoffHz = 18000.0f + tt * ( 80.0f - 18000.0f );
			float cutoffNorm = cutoffHz / plaits::kSampleRate;
			CONSTRAIN( cutoffNorm, 0.001f, 0.49f );
			v.lpFilter.set_f_q<stmlib::FREQUENCY_FAST>( cutoffNorm, 0.6f );
			v.cachedFilterParam = filterParam;
		}
		v.lpFilter.Process<stmlib::FILTER_MODE_LOW_PASS>( buf, buf, numFrames );
	}
	else if ( filterParam > 0 )
	{
		if ( filterParam != v.cachedFilterParam )
		{
			float t = filterParam * 0.01f;
			float tt = t * t;
			float cutoffHz = 20.0f + tt * ( 8000.0f - 20.0f );
			float cutoffNorm = cutoffHz / plaits::kSampleRate;
			CONSTRAIN( cutoffNorm, 0.001f, 0.49f );
			v.hpFilter.set_f_q<stmlib::FREQUENCY_FAST>( cutoffNorm, 0.6f );
			v.cachedFilterParam = filterParam;
		}
		v.hpFilter.Process<stmlib::FILTER_MODE_HIGH_PASS>( buf, buf, numFrames );
	}

	ApplyWavefolder( buf, numFrames, foldParam * 0.01f, foldType );

	if ( driveParam > 0 )
	{
		memcpy( dryBuf, buf, numFrames * sizeof(float) );

		if ( driveParam <= 50 )
		{
			// 0-50%: crossfade dry->Overdrive, with Overdrive's own drive
			// amount also ramping 0->1 across the same range. Using the
			// same small fraction for both the crossfade mix AND
			// Overdrive's drive input is what fixes the reported "volume
			// dips as soon as it's switched on" bug: Overdrive's own gain
			// compensation curve is quietest exactly when driveParam (and
			// therefore the mix amount) is small, so the output stays
			// close to full dry level at low settings instead of jumping
			// to a quiet fully-wet signal.
			float stage1 = driveParam * 0.02f;	// driveParam/50, 0..1
			v.drive.Process( stage1, buf, numFrames );
			for ( int i=0; i<numFrames; ++i )
				buf[i] = dryBuf[i] + stage1 * ( buf[i] - dryBuf[i] );
		}
		else
		{
			// 50-100%: model 1 (Overdrive) is held at max drive - "maxed
			// out at 50%" - and crossfades into model 2 (HardClip, harsher).
			float stage2 = ( driveParam - 50 ) * 0.02f;
			v.drive.Process( 1.0f, buf, numFrames );
			for ( int i=0; i<numFrames; ++i )
			{
				float model1 = buf[i];
				float model2 = HardClip( dryBuf[i] );
				buf[i] = model1 + stage2 * ( model2 - model1 );
			}
		}
	}

	float vol = volParam * 0.01f;
	for ( int i=0; i<numFrames; ++i )
		buf[i] *= vol;

	ApplyCompressor( v, buf, numFrames, compParam );
}

// Writes a voice's rendered mono buffer to its output bus(es). In Stereo
// mode, `outBus` is the left channel and `outBus + 1` is the right, split
// with a simple linear pan law (deliberately cheap/no libm, not equal-
// power - a small centre-level dip is an acceptable tradeoff here). Pass
// `scratch = NULL` to just silence the output (the idle-skip case).
static void WriteVoiceOutput( float* busFrames, int numFrames, const float* scratch, int outBus, bool replace, bool stereo, int panParam )
{
	if ( outBus >= kNT_lastBus )
		stereo = false;	// no room for the R channel one bus higher - fall back to mono rather than overrun

	float* outL = busFrames + ( outBus - 1 ) * numFrames;
	float* outR = stereo ? busFrames + outBus * numFrames : NULL;

	if ( scratch == NULL )
	{
		if ( replace )
		{
			memset( outL, 0, numFrames * sizeof(float) );
			if ( outR ) memset( outR, 0, numFrames * sizeof(float) );
		}
		return;
	}

	if ( !stereo )
	{
		if ( replace ) memcpy( outL, scratch, numFrames * sizeof(float) );
		else for ( int i=0; i<numFrames; ++i ) outL[i] += scratch[i];
		return;
	}

	float pan01 = ( panParam + 100 ) * 0.005f;	// -100..100 -> 0..1
	CONSTRAIN( pan01, 0.0f, 1.0f );
	float gainL = 1.0f - pan01;
	float gainR = pan01;
	if ( replace )
	{
		for ( int i=0; i<numFrames; ++i )
		{
			outL[i] = scratch[i] * gainL;
			outR[i] = scratch[i] * gainR;
		}
	}
	else
	{
		for ( int i=0; i<numFrames; ++i )
		{
			outL[i] += scratch[i] * gainL;
			outR[i] += scratch[i] * gainR;
		}
	}
}

// ---------------------------------------------------------------------
// Sample layer - mixes a loaded WAV into a voice's rendered signal, before
// ApplyPost() (per the explicit "mix should happen early on... so that the
// postfx still applies to the mix as well" spec). See kParamSampleBD,
// kParamSampleMixBD, and kParamKnockTailBD's comments for the 3 controls'
// exact behaviour, and _sampleSlot for the loaded-sample state itself.
// ---------------------------------------------------------------------

static const int kSampleMixParam[kNumSlots] = { kParamSampleMixBD, kParamSampleMixSD, kParamSampleMixCH, kParamSampleMixOH };
static const int kKnockTailParam[kNumSlots] = { kParamKnockTailBD, kParamKnockTailSD, kParamKnockTailCH, kParamKnockTailOH };
static const int kMixTypeParam[kNumSlots]   = { kParamMixTypeBD, kParamMixTypeSD, kParamMixTypeCH, kParamMixTypeOH };

// At trigger time, extends the voice's own idle-tail budget (see
// IdleSamples()) to also cover the loaded sample's remaining playback
// length when one is mixed in - otherwise a short synth release could cut
// the sample's tail off early, since the idle-skip check returns before
// MixSampleLayer() ever runs. A no-op when no sample would play.
static void ExtendIdleForSample( _drumMachineAlgorithm* pThis, _drumVoicePost& v, int slot )
{
	_sampleSlot& s = pThis->sampleSlot[slot];
	if ( !s.hasSample || pThis->v[ kSampleMixParam[slot] ] <= 0 || SampleSystemBusy( pThis ) )
		return;
	float inc = s.loadedSampleRate / (float)NT_globals.sampleRate;
	if ( inc <= 0.0f )
		return;
	int sampleTailFrames = (int)( (float)s.loadedNumFrames / inc );
	if ( sampleTailFrames > v.samplesUntilIdle )
		v.samplesUntilIdle = sampleTailFrames;
}

// Mixes a slot's loaded sample into `scratch` (already holding the synth
// voice's rendered output this block) - a no-op (skips the entire sample-
// player path, not just a silent pass) when no sample is loaded, Sample
// Mix is at 0, or the sample system is still busy/settling after some
// loading-related activity (see SampleSystemBusy()) - confirmed by testing
// that calling into the sample-streaming API while a preset load's own SD
// activity is still in flight froze the device, specifically when actively
// playing (not when idle), so hits are simply not sample-layered at all
// during that window rather than risk the same collision. On trigger,
// (re)opens this slot's stream from the top (see OpenSampleStream()) -
// playback itself is pulled from the disting NT streaming API
// (NT_streamRender(), into NT_globals.workBuffer - transient scratch valid
// only for this step() call, same as the official sampleStreamer.cpp
// example uses it) rather than read from a pre-loaded buffer; `pos`/`inc`
// still track the equivalent *native-file* playback position ourselves
// (for the Knock/Tail split-point comparison below) even though the actual
// resampling now happens inside NT_streamRender(). Reuses
// pThis->dryScratch/elementsExciteScratch as working buffers for the
// windowed/highpassed sample signal - both are free at this point in
// ProcessKick/Snare/Hat.
static void MixSampleLayer( _drumMachineAlgorithm* pThis, _drumVoicePost& v, int slot, bool trig, float accent, float* scratch, int numFrames )
{
	_sampleSlot& s = pThis->sampleSlot[slot];
	int mixValue = pThis->v[ kSampleMixParam[slot] ];
	if ( !s.hasSample || mixValue <= 0 || s.loadedNumFrames < 2 || SampleSystemBusy( pThis ) )
		return;

	if ( trig )
	{
		v.samplePos = 0.0f;
		OpenSampleStream( pThis, slot, accent );
	}

	if ( NT_globals.workBufferSizeBytes < (uint32_t)numFrames * sizeof(_NT_frame) )
		return;	// not enough transient scratch this block - skip rather than risk an overrun
	_NT_frame* renderBuffer = (_NT_frame*)NT_globals.workBuffer;

	float inc = s.loadedSampleRate / (float)NT_globals.sampleRate;
	uint32_t framesRendered = NT_streamRender( s.stream, renderBuffer, (uint32_t)numFrames, inc );

	// 5-zone knock/tail curve - see kParamKnockTailBD's comment for the
	// exact spec: 0=full sample; 0.25=knock only; 0.25-0.5=highpass fades
	// in (still knock only); 0.5-0.75=crossfades to the tail with the
	// highpass held; 0.75-1.0=highpass fades back out, tail only.
	int mixType = pThis->v[ kMixTypeParam[slot] ];
	int splitFrame = ( mixType == kMixTypeEnvFollower ) ? s.splitFrameEnvFollower : s.splitFrameFixed;
	float knobT = pThis->v[ kKnockTailParam[slot] ] * 0.01f;
	float preGain, postGain, hpAmount;
	if ( knobT <= 0.25f )
	{
		preGain = 1.0f;
		postGain = 1.0f - knobT * 4.0f;
		hpAmount = 0.0f;
	}
	else if ( knobT <= 0.5f )
	{
		preGain = 1.0f;
		postGain = 0.0f;
		hpAmount = ( knobT - 0.25f ) * 4.0f;
	}
	else if ( knobT <= 0.75f )
	{
		float t = ( knobT - 0.5f ) * 4.0f;
		preGain = 1.0f - t;
		postGain = t;
		hpAmount = 1.0f;
	}
	else
	{
		preGain = 0.0f;
		postGain = 1.0f;
		hpAmount = 1.0f - ( knobT - 0.75f ) * 4.0f;
	}

	float declick = 0.005f * s.loadedSampleRate;	// ~5ms declick window around the split point, avoids a click from the pre/post gain step
	if ( declick < 1.0f ) declick = 1.0f;

	float* sampleBuf = pThis->dryScratch;
	float* hpBuf = pThis->elementsExciteScratch;

	float pos = v.samplePos;
	int lastValid = (int)s.loadedNumFrames - 1;
	for ( int i=0; i<numFrames; ++i )
	{
		float raw = 0.0f;
		if ( i < (int)framesRendered && pos < (float)lastValid )
		{
			raw = renderBuffer[i][0];
			pos += inc;
		}

		float edge = ( pos - (float)splitFrame + declick ) / ( 2.0f * declick );
		CONSTRAIN( edge, 0.0f, 1.0f );
		float gain = preGain + ( postGain - preGain ) * edge;
		sampleBuf[i] = raw * gain;
	}
	v.samplePos = pos;

	if ( hpAmount > 0.0f )
	{
		memcpy( hpBuf, sampleBuf, numFrames * sizeof(float) );
		v.sampleHp.Process<stmlib::FILTER_MODE_HIGH_PASS>( hpBuf, hpBuf, numFrames );
		for ( int i=0; i<numFrames; ++i )
			sampleBuf[i] += ( hpBuf[i] - sampleBuf[i] ) * hpAmount;
	}

	float mix = mixValue * 0.01f;
	if ( mix <= 0.5f )
	{
		// 0-50%: sample fades in additively under the synth voice, which
		// stays at full level throughout this range.
		float sampleGain = mix * 2.0f;
		for ( int i=0; i<numFrames; ++i )
			scratch[i] += sampleBuf[i] * sampleGain;
	}
	else
	{
		// 50-100%: crossfades the synth voice out until only the sample
		// remains (sample is already at full gain from the first branch).
		float synthGain = 1.0f - ( mix - 0.5f ) * 2.0f;
		for ( int i=0; i<numFrames; ++i )
			scratch[i] = scratch[i] * synthGain + sampleBuf[i];
	}
}

static void ProcessKick( _drumMachineAlgorithm* pThis, float* busFrames, int numFrames )
{
	_kickVoice& v = pThis->dtc->kick;
	bool trig = v.pendingTrigger;
	v.pendingTrigger = false;
	float baseDecay = Smoothed( pThis, kParamRelBD ) * 0.01f;
	if ( trig )
	{
		v.samplesUntilIdle = IdleSamples( baseDecay );	// unmodulated - just needs to be safely long enough
		ExtendIdleForSample( pThis, v, kSlotBD );
	}

	// Envelopes advance every block regardless of the voice's own idle-skip
	// state (below), so they always run to completion and the bar-page
	// overlay never shows a stale/frozen level - cheap either way (a few
	// comparisons, no Render() cost).
	float blockSeconds = numFrames / plaits::kSampleRate;
	float env1Level = AdvanceEnvelope( v.env1, trig, v.pendingAccent, pThis->v[kParamEnv1Morph], pThis->v[kParamEnv1Shape], blockSeconds );
	float env2Level = AdvanceEnvelope( v.env2, trig, v.pendingAccent, pThis->v[kParamEnv2Morph], pThis->v[kParamEnv2Shape], blockSeconds );
	pThis->currentEnv1Level[kSlotBD] = env1Level;
	pThis->currentEnv2Level[kSlotBD] = env2Level;

	bool replace = pThis->v[kParamOutBDMode];
	bool stereo = pThis->v[kParamStereoBD] != 0;

	if ( v.samplesUntilIdle <= 0 && !trig )
	{
		WriteVoiceOutput( busFrames, numFrames, NULL, pThis->v[kParamOutBD], replace, stereo, pThis->v[kParamPanBD] );
		return;
	}

	float* scratch = pThis->renderScratch;
	int pitchIdx = RoundToInt( ModulatedViaMatrix( pThis, kConceptPitch, kSlotBD, Smoothed( pThis, kParamPitchBD ), env1Level, env2Level ) );
	CONSTRAIN( pitchIdx, 0, 127 );
	float f0 = kMidiNoteFreq[ pitchIdx ] / plaits::kSampleRate;
	float decay = ModulatedViaMatrix( pThis, kConceptRelease, kSlotBD, Smoothed( pThis, kParamRelBD ), env1Level, env2Level ) * 0.01f;
	float tone = ModulatedViaMatrix( pThis, kConceptTone, kSlotBD, Smoothed( pThis, kParamToneBD ), env1Level, env2Level ) * 0.01f;
	float character = ModulatedViaMatrix( pThis, kConceptCharacter, kSlotBD, Smoothed( pThis, kParamCharBD ), env1Level, env2Level ) * 0.01f;
	// FM: self-contained internal modulation depth, no audio input - drives
	// the attack pitch-knock on Analog/Synthetic/808 (see kParamFmModeBD
	// below for *how* it's generated) and the core FM amount on the FM
	// model itself. A no-op on Elements.
	float fm = ModulatedViaMatrix( pThis, kConceptFm, kSlotBD, Smoothed( pThis, kParamFmBD ), env1Level, env2Level ) * 0.01f;
	CONSTRAIN( decay, 0.0f, 1.0f );
	CONSTRAIN( tone, 0.0f, 1.0f );
	CONSTRAIN( character, 0.0f, 1.0f );
	CONSTRAIN( fm, 0.0f, 1.0f );
	float accent = v.pendingAccent;
	// FM Mode: how the FM knob's knock energy is generated - see
	// kFmModeXxx's comment. Not modulatable (no Mod Matrix entry, like
	// Model), so read directly rather than through ModulatedViaMatrix().
	int fmMode = ResolveFmMode( pThis->v[kParamFmModeBD] );
	// Analog/Synthetic/808's knock math can push their internal oscillator/
	// resonator frequency to (or past) zero right at the very top of the FM
	// knob's range, which reads as a broken/silent knock rather than an
	// intense one - capping the knob's effective ceiling below that danger
	// zone keeps the exact curve shape (and sound) everywhere it's actually
	// reachable, rather than clamping the DSP math directly (which changed
	// the knock's character even at moderate settings - see each model's
	// header comment). FmDrum's own set_fm_amount is unaffected (uses fm).
	float fmKnock = fm * 0.8f;

	if ( pThis->v[kParamModelBD] == 0 )
	{
		// attack_fm_amount is Plaits' own name for exactly the "knock" the
		// user wants FM to control (a brief pitch-up burst right at the
		// attack, the classic 808/909 kick trick) - now driven by FM
		// directly instead of splitting Character's range across it and
		// self_fm_amount (a separate, ongoing self-FM "growl" during the
		// body, which Character now controls over its full range instead).
		v.analog.Render( trig, accent, f0, tone, decay, fmKnock, character, fmMode, scratch, numFrames );
	}
	else if ( pThis->v[kParamModelBD] == 1 )
	{
		// fm_envelope_amount here is the same attack-knock idea (fm_ starts
		// at 1.0 on trigger and decays, modulating pitch) - already wired to
		// FM directly, so Synthetic gets the same knock with no change.
		v.synthetic.Render( false, trig, accent, f0, tone, decay, character, fmKnock, 0.5f, fmMode, scratch, numFrames );
	}
	else if ( pThis->v[kParamModelBD] == 2 )
	{
		// See ComputeExternalFmPush()'s comment - Elements has no internal
		// per-sample FM hook, so FM pushes its resonator frequency externally,
		// once per block, instead.
		float fmPush = ComputeExternalFmPush( v, trig, accent, fmMode, blockSeconds );
		float modulatedF0 = f0 * ( 1.0f + fmPush * fm * 1.5f );
		CONSTRAIN( modulatedF0, 0.001f, 0.49f );
		RenderElements( v.elementsExciter, v.elementsResonator, pThis->elementsExciteScratch, trig, accent, modulatedF0, tone, decay, character, scratch, numFrames );
		UpdateFmFeedback( v, scratch, numFrames );
	}
	else if ( pThis->v[kParamModelBD] == 3 )
	{
		// 808-style bass drum (Peaks BassDrum / Braids Kick - the same
		// algorithm, see third_party/mi_peaks/ATTRIBUTION.md) - Character
		// drives punch (dynamic Q push on the resonant body); FM drives the
		// attack pitch-knock (originally a fixed, un-knobbed 17-semitone
		// push - see bass_drum.h), same knock the Analog/Synthetic models'
		// own FM input already gives.
		v.peaksBass.set_frequency( pitchIdx << 7 );
		v.peaksBass.set_decay( (uint16_t)( decay * 65535.0f ) );
		v.peaksBass.set_tone( (uint16_t)( tone * 65535.0f ) );
		v.peaksBass.set_punch( (uint16_t)( character * 65535.0f ) );
		v.peaksBass.set_attack_fm_amount( (uint16_t)( fmKnock * 65535.0f ) );
		v.peaksBass.set_fm_mode( fmMode );
		v.peaksBass.Process( trig, accent, scratch, numFrames );
		for ( int i=0; i<numFrames; ++i ) scratch[i] *= accent;
	}
	else
	{
		// Sine-FM drum (Peaks FmDrum, "similar to the BD/SD in Anushri") -
		// FM knob finally drives a literal FM-amount parameter; Character
		// drives the noise/overdrive blend; Tone is a no-op here (this model
		// has no separate tone control of its own).
		v.peaksFm.set_frequency( pitchIdx << 7 );
		v.peaksFm.set_decay( (uint16_t)( decay * 65535.0f ) );
		v.peaksFm.set_fm_amount( (uint16_t)( fm * 65535.0f ) );
		v.peaksFm.set_noise( (uint16_t)( character * 65535.0f ) );
		v.peaksFm.Process( trig, kMidiNoteFreq, plaits::kSampleRate, scratch, numFrames );
		for ( int i=0; i<numFrames; ++i ) scratch[i] *= accent;
	}

	MixSampleLayer( pThis, v, kSlotBD, trig, accent, scratch, numFrames );

	int filtParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptFilter, kSlotBD, Smoothed( pThis, kParamFiltBD ), env1Level, env2Level ) );
	int foldParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptFold, kSlotBD, Smoothed( pThis, kParamFoldBD ), env1Level, env2Level ) );
	int compParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptCompressor, kSlotBD, Smoothed( pThis, kParamCompBD ), env1Level, env2Level ) );
	int driveParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptDrive, kSlotBD, Smoothed( pThis, kParamDriveBD ), env1Level, env2Level ) );
	int volParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptVolume, kSlotBD, Smoothed( pThis, kParamVolBD ), env1Level, env2Level ) );
	CONSTRAIN( filtParam, -100, 100 );
	CONSTRAIN( foldParam, 0, 100 );
	CONSTRAIN( compParam, 0, 100 );
	CONSTRAIN( driveParam, 0, 100 );
	CONSTRAIN( volParam, 0, 200 );
	ApplyPost( v, scratch, pThis->dryScratch, numFrames, filtParam, foldParam, pThis->v[kParamFoldTypeBD], compParam, driveParam, volParam );

	v.samplesUntilIdle -= numFrames;
	if ( v.samplesUntilIdle < 0 ) v.samplesUntilIdle = 0;

	WriteVoiceOutput( busFrames, numFrames, scratch, pThis->v[kParamOutBD], replace, stereo, pThis->v[kParamPanBD] );
}

static void ProcessSnare( _drumMachineAlgorithm* pThis, float* busFrames, int numFrames )
{
	_snareVoice& v = pThis->dtc->snare;
	bool trig = v.pendingTrigger;
	v.pendingTrigger = false;
	float baseDecay = Smoothed( pThis, kParamRelSD ) * 0.01f;
	if ( trig )
	{
		v.samplesUntilIdle = IdleSamples( baseDecay );
		ExtendIdleForSample( pThis, v, kSlotSD );
	}

	float blockSeconds = numFrames / plaits::kSampleRate;
	float env1Level = AdvanceEnvelope( v.env1, trig, v.pendingAccent, pThis->v[kParamEnv1Morph], pThis->v[kParamEnv1Shape], blockSeconds );
	float env2Level = AdvanceEnvelope( v.env2, trig, v.pendingAccent, pThis->v[kParamEnv2Morph], pThis->v[kParamEnv2Shape], blockSeconds );
	pThis->currentEnv1Level[kSlotSD] = env1Level;
	pThis->currentEnv2Level[kSlotSD] = env2Level;

	bool replace = pThis->v[kParamOutSDMode];
	bool stereo = pThis->v[kParamStereoSD] != 0;

	if ( v.samplesUntilIdle <= 0 && !trig )
	{
		WriteVoiceOutput( busFrames, numFrames, NULL, pThis->v[kParamOutSD], replace, stereo, pThis->v[kParamPanSD] );
		return;
	}

	float* scratch = pThis->renderScratch;
	int pitchIdx = RoundToInt( ModulatedViaMatrix( pThis, kConceptPitch, kSlotSD, Smoothed( pThis, kParamPitchSD ), env1Level, env2Level ) );
	CONSTRAIN( pitchIdx, 0, 127 );
	float f0 = kMidiNoteFreq[ pitchIdx ] / plaits::kSampleRate;
	float decay = ModulatedViaMatrix( pThis, kConceptRelease, kSlotSD, Smoothed( pThis, kParamRelSD ), env1Level, env2Level ) * 0.01f;
	float tone = ModulatedViaMatrix( pThis, kConceptTone, kSlotSD, Smoothed( pThis, kParamToneSD ), env1Level, env2Level ) * 0.01f;
	float character = ModulatedViaMatrix( pThis, kConceptCharacter, kSlotSD, Smoothed( pThis, kParamCharSD ), env1Level, env2Level ) * 0.01f;
	// FM: self-contained internal modulation, no audio input. Only the
	// Synthetic model has a free FM-capable input here - its `fm_amount`
	// parameter (confirmed by reading synthetic_snare_drum.h's actual
	// Render() signature: (sustain, trigger, accent, f0, fm_amount, decay,
	// snappy) - note this model has NO tone input at all, unlike Analog, so
	// Tone is simply not passed to it below). Analog snare's Render() takes
	// (trigger, accent, f0, tone, decay, snappy) with no separate FM slot,
	// so FM is a no-op there.
	float fm = ModulatedViaMatrix( pThis, kConceptFm, kSlotSD, Smoothed( pThis, kParamFmSD ), env1Level, env2Level ) * 0.01f;
	CONSTRAIN( decay, 0.0f, 1.0f );
	CONSTRAIN( tone, 0.0f, 1.0f );
	CONSTRAIN( character, 0.0f, 1.0f );
	CONSTRAIN( fm, 0.0f, 1.0f );
	float accent = v.pendingAccent;
	int fmMode = ResolveFmMode( pThis->v[kParamFmModeSD] );
	// See ProcessKick()'s matching comment - Synthetic's knock math can push
	// its phase increment through 0 right at the very top of the FM knob's
	// range.
	float fmKnock = fm * 0.8f;

	if ( pThis->v[kParamModelSD] == 0 )
		v.analog.Render( trig, accent, f0, tone, decay, character, scratch, numFrames );
	else if ( pThis->v[kParamModelSD] == 1 )
		v.synthetic.Render( false, trig, accent, f0, fmKnock, decay, character, fmMode, scratch, numFrames );
	else if ( pThis->v[kParamModelSD] == 2 )
	{
		float fmPush = ComputeExternalFmPush( v, trig, accent, fmMode, blockSeconds );
		float modulatedF0 = f0 * ( 1.0f + fmPush * fm * 1.5f );
		CONSTRAIN( modulatedF0, 0.001f, 0.49f );
		RenderElements( v.elementsExciter, v.elementsResonator, pThis->elementsExciteScratch, trig, accent, modulatedF0, tone, decay, character, scratch, numFrames );
		UpdateFmFeedback( v, scratch, numFrames );
	}
	else if ( pThis->v[kParamModelSD] == 3 )
	{
		// 808-style snare (Peaks SnareDrum / Braids Snare - same algorithm,
		// see third_party/mi_peaks/ATTRIBUTION.md) - Character drives snappy
		// (noise-band amount).
		v.peaksSnare.set_frequency( pitchIdx << 7 );
		v.peaksSnare.set_decay( (uint16_t)( decay * 65535.0f ) );
		v.peaksSnare.set_tone( (uint16_t)( tone * 65535.0f ) );
		v.peaksSnare.set_snappy( (uint16_t)( character * 65535.0f ) );
		v.peaksSnare.Process( trig, scratch, numFrames );
		for ( int i=0; i<numFrames; ++i ) scratch[i] *= accent;
	}
	else
	{
		// Sine-FM drum (Peaks FmDrum) - same as the BD FM model, its own
		// per-voice instance so BD/SD's self-FM sweeps never interact.
		v.peaksFm.set_frequency( pitchIdx << 7 );
		v.peaksFm.set_decay( (uint16_t)( decay * 65535.0f ) );
		v.peaksFm.set_fm_amount( (uint16_t)( fm * 65535.0f ) );
		v.peaksFm.set_noise( (uint16_t)( character * 65535.0f ) );
		v.peaksFm.Process( trig, kMidiNoteFreq, plaits::kSampleRate, scratch, numFrames );
		for ( int i=0; i<numFrames; ++i ) scratch[i] *= accent;
	}

	MixSampleLayer( pThis, v, kSlotSD, trig, accent, scratch, numFrames );

	int filtParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptFilter, kSlotSD, Smoothed( pThis, kParamFiltSD ), env1Level, env2Level ) );
	int foldParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptFold, kSlotSD, Smoothed( pThis, kParamFoldSD ), env1Level, env2Level ) );
	int compParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptCompressor, kSlotSD, Smoothed( pThis, kParamCompSD ), env1Level, env2Level ) );
	int driveParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptDrive, kSlotSD, Smoothed( pThis, kParamDriveSD ), env1Level, env2Level ) );
	int volParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptVolume, kSlotSD, Smoothed( pThis, kParamVolSD ), env1Level, env2Level ) );
	CONSTRAIN( filtParam, -100, 100 );
	CONSTRAIN( foldParam, 0, 100 );
	CONSTRAIN( compParam, 0, 100 );
	CONSTRAIN( driveParam, 0, 100 );
	CONSTRAIN( volParam, 0, 200 );
	ApplyPost( v, scratch, pThis->dryScratch, numFrames, filtParam, foldParam, pThis->v[kParamFoldTypeSD], compParam, driveParam, volParam );

	v.samplesUntilIdle -= numFrames;
	if ( v.samplesUntilIdle < 0 ) v.samplesUntilIdle = 0;

	WriteVoiceOutput( busFrames, numFrames, scratch, pThis->v[kParamOutSD], replace, stereo, pThis->v[kParamPanSD] );
}

static void ProcessHat( _drumMachineAlgorithm* pThis, float* busFrames, int numFrames, bool isOpen )
{
	_hatVoice& v = isOpen ? pThis->dtc->openHat : pThis->dtc->closedHat;
	int outParamIdx = isOpen ? kParamOutOH : kParamOutCH;
	int modeParamIdx = isOpen ? kParamOutOHMode : kParamOutCHMode;
	int pitchParamIdx = isOpen ? kParamPitchOH : kParamPitchCH;
	int relParamIdx = isOpen ? kParamRelOH : kParamRelCH;
	int toneParamIdx = isOpen ? kParamToneOH : kParamToneCH;
	int charParamIdx = isOpen ? kParamCharOH : kParamCharCH;
	int filtParamIdx = isOpen ? kParamFiltOH : kParamFiltCH;
	int foldParamIdx = isOpen ? kParamFoldOH : kParamFoldCH;
	int foldTypeParamIdx = isOpen ? kParamFoldTypeOH : kParamFoldTypeCH;
	int compParamIdx = isOpen ? kParamCompOH : kParamCompCH;
	int driveParamIdx = isOpen ? kParamDriveOH : kParamDriveCH;
	int volParamIdx = isOpen ? kParamVolOH : kParamVolCH;
	int stereoParamIdx = isOpen ? kParamStereoOH : kParamStereoCH;
	int panParamIdx = isOpen ? kParamPanOH : kParamPanCH;
	int modelParamIdx = isOpen ? kParamModelOH : kParamModelCH;
	int fmParamIdx = isOpen ? kParamFmOH : kParamFmCH;
	int fmModeParamIdx = isOpen ? kParamFmModeOH : kParamFmModeCH;

	int slot = isOpen ? kSlotOH : kSlotCH;

	bool trig = v.pendingTrigger;
	v.pendingTrigger = false;
	float baseDecay = Smoothed( pThis, relParamIdx ) * 0.01f;
	if ( trig )
	{
		v.samplesUntilIdle = IdleSamples( baseDecay );
		ExtendIdleForSample( pThis, v, slot );
	}

	float blockSeconds = numFrames / plaits::kSampleRate;
	float env1Level = AdvanceEnvelope( v.env1, trig, v.pendingAccent, pThis->v[kParamEnv1Morph], pThis->v[kParamEnv1Shape], blockSeconds );
	float env2Level = AdvanceEnvelope( v.env2, trig, v.pendingAccent, pThis->v[kParamEnv2Morph], pThis->v[kParamEnv2Shape], blockSeconds );
	pThis->currentEnv1Level[slot] = env1Level;
	pThis->currentEnv2Level[slot] = env2Level;

	bool replace = pThis->v[modeParamIdx];
	bool stereo = pThis->v[stereoParamIdx] != 0;

	if ( v.samplesUntilIdle <= 0 && !trig )
	{
		WriteVoiceOutput( busFrames, numFrames, NULL, pThis->v[outParamIdx], replace, stereo, pThis->v[panParamIdx] );
		return;
	}

	float* scratch = pThis->renderScratch;
	int pitchIdx = RoundToInt( ModulatedViaMatrix( pThis, kConceptPitch, slot, Smoothed( pThis, pitchParamIdx ), env1Level, env2Level ) );
	CONSTRAIN( pitchIdx, 0, 127 );
	float f0 = kMidiNoteFreq[ pitchIdx ] / plaits::kSampleRate;
	float decay = ModulatedViaMatrix( pThis, kConceptRelease, slot, Smoothed( pThis, relParamIdx ), env1Level, env2Level ) * 0.01f;
	float tone = ModulatedViaMatrix( pThis, kConceptTone, slot, Smoothed( pThis, toneParamIdx ), env1Level, env2Level ) * 0.01f;
	float noisiness = ModulatedViaMatrix( pThis, kConceptCharacter, slot, Smoothed( pThis, charParamIdx ), env1Level, env2Level ) * 0.01f;
	// FM: pushes pitch on all 3 models here - see ComputeExternalFmPush()'s
	// comment (none of them have an internal per-sample FM hook the way
	// Analog/Synthetic/808 do, so it's applied externally, once per block).
	float fm = ModulatedViaMatrix( pThis, kConceptFm, slot, Smoothed( pThis, fmParamIdx ), env1Level, env2Level ) * 0.01f;
	CONSTRAIN( decay, 0.0f, 1.0f );
	CONSTRAIN( tone, 0.0f, 1.0f );
	CONSTRAIN( noisiness, 0.0f, 1.0f );
	CONSTRAIN( fm, 0.0f, 1.0f );
	float accent = v.pendingAccent;
	int fmMode = ResolveFmMode( pThis->v[fmModeParamIdx] );

	if ( pThis->v[modelParamIdx] == 0 )
	{
		float fmPush = ComputeExternalFmPush( v, trig, accent, fmMode, blockSeconds );
		float modulatedF0 = f0 * ( 1.0f + fmPush * fm * 1.5f );
		CONSTRAIN( modulatedF0, 0.001f, 0.49f );
		v.hihat.Render( trig, accent, modulatedF0, tone, decay, noisiness, v.scratch1, v.scratch2, scratch, numFrames );
		UpdateFmFeedback( v, scratch, numFrames );
	}
	else if ( pThis->v[modelParamIdx] == 1 )
	{
		float fmPush = ComputeExternalFmPush( v, trig, accent, fmMode, blockSeconds );
		float modulatedF0 = f0 * ( 1.0f + fmPush * fm * 1.5f );
		CONSTRAIN( modulatedF0, 0.001f, 0.49f );
		RenderElements( v.elementsExciter, v.elementsResonator, pThis->elementsExciteScratch, trig, accent, modulatedF0, tone, decay, noisiness, scratch, numFrames );
		UpdateFmFeedback( v, scratch, numFrames );
	}
	else
	{
		// Braids' cymbal/hi-hat model has no envelope of its own (a
		// continuously-running metallic/noise texture - see
		// third_party/mi_braids/braids/cymbal.h) - this voice supplies a
		// simple linear decay envelope itself, timed off the same Release
		// knob as every other model, and applies it as a post-multiply.
		if ( trig )
		{
			v.cymbalElapsed = 0.0f;
			v.cymbalTotalS = NormToSeconds( decay * decay, 0.05f, 2.0f );
			v.cymbalActive = true;
		}
		float cymbalAmp = 0.0f;
		if ( v.cymbalActive )
		{
			v.cymbalElapsed += blockSeconds;
			if ( v.cymbalElapsed >= v.cymbalTotalS )
				v.cymbalActive = false;
			else
				cymbalAmp = 1.0f - v.cymbalElapsed / v.cymbalTotalS;
		}
		// FM pushes pitch here too (Q7 units, ~1 semitone per unit of
		// fmPush*fm*18 below) - see ComputeExternalFmPush()'s comment.
		float fmPush = ComputeExternalFmPush( v, trig, accent, fmMode, blockSeconds );
		int32_t pitchPush = (int32_t)( fmPush * fm * 18.0f * 128.0f );
		v.braidsCymbal.set_pitch( ( pitchIdx << 7 ) + pitchPush );
		v.braidsCymbal.set_tone( (uint16_t)( tone * 65535.0f ) );
		v.braidsCymbal.set_xfade( (uint16_t)( noisiness * 65535.0f ) );
		v.braidsCymbal.Process( kMidiNoteFreq, plaits::kSampleRate, scratch, numFrames );
		float amp = cymbalAmp * accent;
		for ( int i=0; i<numFrames; ++i ) scratch[i] *= amp;
		UpdateFmFeedback( v, scratch, numFrames );
	}

	MixSampleLayer( pThis, v, slot, trig, accent, scratch, numFrames );

	int filtParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptFilter, slot, Smoothed( pThis, filtParamIdx ), env1Level, env2Level ) );
	int foldParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptFold, slot, Smoothed( pThis, foldParamIdx ), env1Level, env2Level ) );
	int compParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptCompressor, slot, Smoothed( pThis, compParamIdx ), env1Level, env2Level ) );
	int driveParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptDrive, slot, Smoothed( pThis, driveParamIdx ), env1Level, env2Level ) );
	int volParam = RoundToInt( ModulatedViaMatrix( pThis, kConceptVolume, slot, Smoothed( pThis, volParamIdx ), env1Level, env2Level ) );
	CONSTRAIN( filtParam, -100, 100 );
	CONSTRAIN( foldParam, 0, 100 );
	CONSTRAIN( compParam, 0, 100 );
	CONSTRAIN( driveParam, 0, 100 );
	CONSTRAIN( volParam, 0, 200 );
	ApplyPost( v, scratch, pThis->dryScratch, numFrames, filtParam, foldParam, pThis->v[foldTypeParamIdx], compParam, driveParam, volParam );

	v.samplesUntilIdle -= numFrames;
	if ( v.samplesUntilIdle < 0 ) v.samplesUntilIdle = 0;

	WriteVoiceOutput( busFrames, numFrames, scratch, pThis->v[outParamIdx], replace, stereo, pThis->v[panParamIdx] );
}

void 	step( _NT_algorithm* self, float* busFrames, int numFramesBy4 )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;
	int numFrames = numFramesBy4 * 4;

	// Algorithms should defer SD-card activity until it's known to be
	// mounted, which might be well after construct() - watch for the mount
	// transition here (matches the official distingNT_API samplePlayer.cpp
	// example) and (re)scan for this instance's BD/SD/CH/OH folders then.
	bool cardMounted = NT_isSdCardMounted();
	if ( cardMounted != pThis->sdCardWasMounted )
	{
		pThis->sdCardWasMounted = cardMounted;
		if ( cardMounted )
			ScanSampleFolders( pThis );
	}
	AdvanceSampleAnalysis( pThis );
	if ( pThis->sampleSystemGraceSamples > 0 )
	{
		pThis->sampleSystemGraceSamples -= numFrames;
		if ( pThis->sampleSystemGraceSamples < 0 ) pThis->sampleSystemGraceSamples = 0;
	}

	AdvanceSmoothers( pThis, numFrames );
	DecayVelocityMeters( pThis, numFrames );
	AdvanceLfos( pThis, numFrames );
	pThis->sampleCounter += numFrames;

	ProcessKick( pThis, busFrames, numFrames );
	ProcessSnare( pThis, busFrames, numFrames );
	ProcessHat( pThis, busFrames, numFrames, false );
	ProcessHat( pThis, busFrames, numFrames, true );
}

// ---------------------------------------------------------------------
// Custom UI
// ---------------------------------------------------------------------

uint32_t	hasCustomUi( _NT_algorithm* self )
{
	return kNT_encoderL | kNT_encoderR | kNT_encoderButtonR | kNT_potL | kNT_potC | kNT_potR
		| kNT_potButtonL | kNT_potButtonC | kNT_potButtonR | kNT_button3 | kNT_button4;
}

static int SetupPageItemCount( int page ) { return kPageItemCount[page]; }
static const uint8_t* SetupPageParams( int page ) { return kPageParams[page]; }

// Sets a parameter from the custom UI and, for the 32 smoothed parameters,
// snaps the smoother's shadow value directly here rather than relying on
// parameterChanged() to do it - parameterChanged()'s callback timing for
// changes originating from our own NT_setParameterFromUi() call isn't
// guaranteed, so this is the one path we fully control ourselves. Guarded
// by loadingPreset so LoadPreset()'s own ramp setup (which calls
// NT_setParameterFromUi() directly, not through here) is never overwritten.
static void SetParam( _drumMachineAlgorithm* pThis, int paramIndex, int value )
{
	const _NT_parameter& p = pThis->parameters[paramIndex];
	if ( value < p.min ) value = p.min;
	if ( value > p.max ) value = p.max;
	NT_setParameterFromUi( NT_algorithmIndex( pThis ), paramIndex + NT_parameterOffset(), value );

	if ( paramIndex >= kFirstSmoothedParam && paramIndex < kFirstSmoothedParam + kNumSmoothedParams && !pThis->loadingPreset )
	{
		_paramSmoother& s = pThis->smoother[ paramIndex - kFirstSmoothedParam ];
		s.current = s.target = (float)value;
		s.increment = 0.0f;
		s.samplesRemaining = 0;
	}
}

// Relative/incremental pot handling (see potHasLastPos/potLastPos's comment
// on the algorithm struct). `rawPos` is the pot's current absolute 0..1
// position. Returns true if the parameter should be updated this call, with
// the new value written to *outValue.
static bool PotRelativeUpdate( _drumMachineAlgorithm* pThis, int potIdx, float rawPos, int paramMin, int paramMax, int currentValue, int* outValue )
{
	// First reading since a reset (page change/preset load/UI-open) must
	// ONLY establish a baseline, never move the parameter - this used to be
	// checked *after* an "at the physical limit, snap to min/max" special
	// case below, which meant any pot merely resting near one end when a
	// page was opened (extremely common - pots get left wherever after use
	// elsewhere) instantly slammed that page's parameter to 0 or 100 on the
	// very first touch. Fixed by checking this first, unconditionally.
	if ( !pThis->potHasLastPos[potIdx] )
	{
		pThis->potLastPos[potIdx] = rawPos;
		pThis->potHasLastPos[potIdx] = true;
		pThis->potAccum[potIdx] = 0.0f;
		return false;
	}

	// No special-cased "snap to min/max at the pot's physical limit" here
	// (an earlier version had one) - real potentiometers commonly saturate
	// to 0.0/1.0 somewhat before their true mechanical end-stop, so that
	// rule fired on ordinary turns near either end, not just deliberate
	// full-travel moves, and felt like the same instant jump this whole
	// function exists to avoid. SetParam() already clamps to [min,max], so
	// the true extremes stay reachable with a deliberate turn; this trades
	// a little reach-the-exact-end friction for never jumping unexpectedly.
	float delta = rawPos - pThis->potLastPos[potIdx];
	pThis->potLastPos[potIdx] = rawPos;

	int range = paramMax - paramMin;

	// Low-cardinality parameters (a handful of discrete options - e.g.
	// Model's 3-way enum, or LFO Rate's 7 divisions) feel broken under the
	// plain `delta * range` scaling below: reaching a full step requires
	// accumulating HALF the pot's *entire* physical travel for a range as
	// small as 2. Since the pot's resting position has no relation to the
	// logical value (see potLastPos's comment), it commonly ends up resting
	// close to one physical end already - leaving less than half a turn of
	// travel available in that direction, so some options become
	// physically unreachable no matter how far you turn, without first
	// "winding up" the other way (not an obvious or discoverable move).
	// Below this threshold, use a fixed "distance per step" in pot-travel
	// terms instead, independent of range, so a comfortable partial turn
	// from *any* resting position reaches every option, and every step is a
	// deliberate, discrete +-1 that never skips over one.
	constexpr int kDetentRangeThreshold = 8;
	if ( range > 0 && range <= kDetentRangeThreshold )
	{
		constexpr float kDetentDistance = 0.12f;	// ~12% of full travel per step
		pThis->potAccum[potIdx] += delta;
		if ( pThis->potAccum[potIdx] >= kDetentDistance )
		{
			pThis->potAccum[potIdx] -= kDetentDistance;
			*outValue = currentValue + 1;
			return true;
		}
		if ( pThis->potAccum[potIdx] <= -kDetentDistance )
		{
			pThis->potAccum[potIdx] += kDetentDistance;
			*outValue = currentValue - 1;
			return true;
		}
		return false;
	}

	// Continuous parameters keep the original proportional scaling - a full
	// careful sweep can reach close to the whole range in one deliberate
	// motion, with the fractional carry (potAccum) preserving any sub-step
	// motion between calls instead of discarding it (needed since these are
	// typically high-resolution pots reporting many small deltas per
	// physical turn).
	pThis->potAccum[potIdx] += delta * (float)range;
	int intDelta = (int)pThis->potAccum[potIdx];	// truncates toward zero
	if ( intDelta == 0 )
		return false;
	pThis->potAccum[potIdx] -= (float)intDelta;
	*outValue = currentValue + intDelta;
	return true;
}

// Splices one word from each list together (e.g. "SHADOW VENOM") into a
// fresh recallable name - called each time a slot is saved, so overwriting
// a slot always gives it a new name reflecting the new take.
static void GenerateRandomPresetName( char* out )
{
	uint32_t a = stmlib::Random::GetWord() % ARRAY_SIZE(kPresetNameListA);
	uint32_t b = stmlib::Random::GetWord() % ARRAY_SIZE(kPresetNameListB);
	strcpy( out, kPresetNameListA[a] );
	strcat( out, " " );
	strcat( out, kPresetNameListB[b] );
}

static void SavePreset( _drumMachineAlgorithm* pThis, int slot )
{
	for ( int i=0; i<kNumKitParams; ++i )
		pThis->presetBank[slot][i] = pThis->v[ kFirstKitParam + i ];
	for ( int i=0; i<kNumModParams; ++i )
		pThis->modBank[slot][i] = pThis->v[ ModParamAt(i) ];
	pThis->presetBankValid[slot] = true;
	GenerateRandomPresetName( pThis->presetName[slot] );
}

// Model parameters snap instantly ("models are loaded directly"); every
// other kit parameter is set instantly too (so the displayed value/bar
// jumps immediately, same as any other edit) but ramps the actual
// DSP-facing smoothed value over ~1.5s - see AdvanceSmoothers()/Smoothed().
static void LoadPreset( _drumMachineAlgorithm* pThis, int slot )
{
	if ( !pThis->presetBankValid[slot] )
		return;

	ExtendSampleSystemGrace( pThis );

	int algIdx = NT_algorithmIndex( pThis );
	uint32_t off = NT_parameterOffset();

	static const int kModelParams[kNumSlots] = { kParamModelBD, kParamModelSD, kParamModelCH, kParamModelOH };
	for ( int i=0; i<kNumSlots; ++i )
	{
		int p = kModelParams[i];
		NT_setParameterFromUi( algIdx, p + off, pThis->presetBank[slot][ p - kFirstKitParam ] );
	}

	pThis->loadingPreset++;
	float rampSamples = 1.5f * plaits::kSampleRate;
	for ( int i=0; i<kNumSmoothedParams; ++i )
	{
		int p = kFirstSmoothedParam + i;
		int newValue = pThis->presetBank[slot][ p - kFirstKitParam ];
		_paramSmoother& s = pThis->smoother[i];
		s.target = (float)newValue;
		s.samplesRemaining = (int)rampSamples;
		s.increment = ( s.target - s.current ) / rampSamples;
		NT_setParameterFromUi( algIdx, p + off, newValue );
	}
	pThis->loadingPreset--;

	// Curves globals + FM + Envelope depths - snap instantly, no fade
	// (matches Model's "instant, no fade" precedent rather than extending
	// the smoothing system to another 140 parameters).
	for ( int i=0; i<kNumModParams; ++i )
	{
		int p = ModParamAt(i);
		NT_setParameterFromUi( algIdx, p + off, pThis->modBank[slot][i] );
	}

	// parameterChanged()'s callback timing for changes originating from our
	// own NT_setParameterFromUi() calls above isn't guaranteed (see
	// SetParam()'s matching comment) - Sample's side effect (kicking off the
	// actual SD card read) can't be left to chance the way the other
	// kModParams entries can (none of which have a load-time side effect),
	// so explicitly (re)start each slot's load here regardless.
	StartSampleLoad( pThis, kSlotBD );
	StartSampleLoad( pThis, kSlotSD );
	StartSampleLoad( pThis, kSlotCH );
	StartSampleLoad( pThis, kSlotOH );

	pThis->potHasLastPos[0] = pThis->potHasLastPos[1] = pThis->potHasLastPos[2] = false;
	pThis->potAccum[0] = pThis->potAccum[1] = pThis->potAccum[2] = 0.0f;
}

// Resolves the route mapping `source` to (concept, slot) for quick-edit
// (button 4) depth adjustment on a bar page - reuses an existing matching
// route if one exists (via the read-only FindRoute()), otherwise claims a
// free (Source=None) route by writing its Source/Slot/Concept (leaving
// Depth at 0) so the pot/encoder delta in customUi() has somewhere to
// write. Returns -1 if no matching route exists AND all kNumModRoutes are
// already claimed by other mappings - that slot's quick-edit control is
// then a no-op until a route frees up elsewhere.
static int FindOrAllocRoute( _drumMachineAlgorithm* pThis, int source, int slot, int concept )
{
	int existing = FindRoute( pThis, source, slot, concept );
	if ( existing >= 0 )
		return existing;

	int freeRoute = -1;
	for ( int r=0; r<kNumModRoutes; ++r )
	{
		if ( pThis->v[ ModRouteParam( r, kModParamSource ) ] == kModSrcNone )
		{
			freeRoute = r;
			break;
		}
	}
	if ( freeRoute < 0 )
		return -1;

	SetParam( pThis, ModRouteParam( freeRoute, kModParamSource ), source );
	SetParam( pThis, ModRouteParam( freeRoute, kModParamSlot ), slot );
	SetParam( pThis, ModRouteParam( freeRoute, kModParamConcept ), concept );
	SetParam( pThis, ModRouteParam( freeRoute, kModParamDepth ), 0 );
	return freeRoute;
}

// Applies one pot's relative delta to control slotIdx's value on the
// current bar page - either the concept's own base parameter, or (in
// quick-edit depth mode) the active source's Mod Matrix depth for that
// slot. Route lookup for the pot's baseline/delta tracking is read-only
// (FindRoute(), same as DrawBarPage()'s display) so merely holding a pot
// over an unassigned slot never silently claims one of the 8 routes -
// only an actual committed change (PotRelativeUpdate() returning true)
// calls the allocating FindOrAllocRoute(). Bug fix: this used to resolve
// the parameter to edit via a single helper that called the *allocating*
// lookup unconditionally on every UI tick a pot was active (not just on an
// actual change), so simply entering a quick-edit mode immediately claimed
// routes for all 4 slots before any knob was ever turned - by the time a
// 3rd/4th source was tried on the same page, all 8 routes were already
// silently used up, so its pots had nothing left to allocate and did
// nothing.
static void ApplyBarPot( _drumMachineAlgorithm* pThis, int potIdx, float rawPos, bool editingDepth, int source, int concept, const uint8_t* baseParams, int slotIdx )
{
	if ( editingDepth )
	{
		int route = FindRoute( pThis, source, slotIdx, concept );
		int currentDepth = route < 0 ? 0 : pThis->v[ ModRouteParam( route, kModParamDepth ) ];
		int newValue;
		if ( !PotRelativeUpdate( pThis, potIdx, rawPos, -100, 100, currentDepth, &newValue ) )
			return;
		int r = FindOrAllocRoute( pThis, source, slotIdx, concept );
		if ( r >= 0 )
			SetParam( pThis, ModRouteParam( r, kModParamDepth ), newValue );
	}
	else
	{
		int paramIndex = baseParams[slotIdx];
		const _NT_parameter& p = pThis->parameters[paramIndex];
		int newValue;
		if ( PotRelativeUpdate( pThis, potIdx, rawPos, p.min, p.max, pThis->v[paramIndex], &newValue ) )
			SetParam( pThis, paramIndex, newValue );
	}
}

// Same idea as ApplyBarPot() but for Encoder R, which reports a ready-made
// relative delta directly (no PotRelativeUpdate()/baseline tracking
// needed) - still only allocates a route once there's an actual nonzero
// delta to commit.
static void ApplyBarEncoder( _drumMachineAlgorithm* pThis, int delta, bool editingDepth, int source, int concept, const uint8_t* baseParams, int slotIdx )
{
	if ( editingDepth )
	{
		int route = FindRoute( pThis, source, slotIdx, concept );
		int currentDepth = route < 0 ? 0 : pThis->v[ ModRouteParam( route, kModParamDepth ) ];
		int newValue = currentDepth + delta;
		CONSTRAIN( newValue, -100, 100 );
		int r = FindOrAllocRoute( pThis, source, slotIdx, concept );
		if ( r >= 0 )
			SetParam( pThis, ModRouteParam( r, kModParamDepth ), newValue );
	}
	else
	{
		int paramIndex = baseParams[slotIdx];
		int current = pThis->v[paramIndex];
		SetParam( pThis, paramIndex, current + delta );
	}
}

void	customUi( _NT_algorithm* self, const _NT_uiData& data )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;

	if ( ( data.controls & kNT_button3 ) && !( data.lastButtons & kNT_button3 ) )
	{
		pThis->presetMenuOpen = !pThis->presetMenuOpen;
		pThis->presetMenuStage = 0;
	}

	if ( pThis->presetMenuOpen )
	{
		if ( pThis->presetMenuStage == 0 )
		{
			// Browsing: Encoder R scrolls the slot list; its push button
			// drills into the Load/Save choice for the highlighted slot.
			if ( data.encoders[1] != 0 )
			{
				pThis->presetMenuIndex += data.encoders[1];
				if ( pThis->presetMenuIndex < 0 ) pThis->presetMenuIndex = 0;
				if ( pThis->presetMenuIndex >= kNumPresets ) pThis->presetMenuIndex = kNumPresets - 1;
			}
			if ( ( data.controls & kNT_encoderButtonR ) && !( data.lastButtons & kNT_encoderButtonR ) )
			{
				pThis->presetMenuStage = 1;
				pThis->presetMenuAction = 0;	// always default to Load - never accidentally overwrite
			}
			return;	// preset menu owns every claimed control while it's open
		}

		// Stage 1: choosing Load/Save for the highlighted slot - still all
		// on Encoder R (rotate toggles the choice, push confirms).
		if ( data.encoders[1] != 0 )
			pThis->presetMenuAction = pThis->presetMenuAction == 0 ? 1 : 0;
		if ( ( data.controls & kNT_encoderButtonR ) && !( data.lastButtons & kNT_encoderButtonR ) )
		{
			if ( pThis->presetMenuAction == 0 )
				LoadPreset( pThis, pThis->presetMenuIndex );
			else
				SavePreset( pThis, pThis->presetMenuIndex );
			pThis->presetMenuOpen = false;
		}
		return;	// preset menu owns every claimed control while it's open
	}

	if ( pThis->deleteConfirmOpen )
	{
		// Same interaction idiom as the preset menu's Load/Save toggle:
		// rotate Encoder R to switch the choice, push to confirm - defaults
		// to No so a stray click when opening the dialog can never delete
		// anything by itself.
		if ( data.encoders[1] != 0 )
			pThis->deleteConfirmChoice = pThis->deleteConfirmChoice == 0 ? 1 : 0;
		if ( ( data.controls & kNT_encoderButtonR ) && !( data.lastButtons & kNT_encoderButtonR ) )
		{
			if ( pThis->deleteConfirmChoice == 1 )
			{
				int concept = kPageConcept[ pThis->currentPage ];
				int route = FindRoute( pThis, pThis->barPageMode, pThis->deleteConfirmSlot, concept );
				if ( route >= 0 )
					SetParam( pThis, ModRouteParam( route, kModParamSource ), kModSrcNone );
			}
			pThis->deleteConfirmOpen = false;
		}
		return;	// delete-confirm dialog owns every claimed control while it's open
	}

	if ( data.encoders[0] != 0 )
	{
		pThis->currentPage += data.encoders[0];
		if ( pThis->currentPage < kFirstCustomPage ) pThis->currentPage = kFirstCustomPage;
		if ( pThis->currentPage >= kNumPages ) pThis->currentPage = kNumPages - 1;
		pThis->setupSelectedItem = 0;
		pThis->setupEditMode = false;
		pThis->barPageMode = kModSrcNone;
		pThis->potHasLastPos[0] = pThis->potHasLastPos[1] = pThis->potHasLastPos[2] = false;
	pThis->potAccum[0] = pThis->potAccum[1] = pThis->potAccum[2] = 0.0f;
	}

	if ( kPageType[pThis->currentPage] == kPageTypeList )
	{
		// List-style page (Routing/MIDI/Mod Matrix): rotate Encoder R to
		// scroll while browsing, push to enter edit mode (rotate now adjusts
		// the highlighted item's value), push again to return to browsing.
		// See setupEditMode's comment on the algorithm struct for why this
		// replaced the old "push advances one item at a time" scheme - that
		// becomes unusable on a 32-item page.
		int itemCount = SetupPageItemCount( pThis->currentPage );
		const uint8_t* params = SetupPageParams( pThis->currentPage );

		if ( ( data.controls & kNT_encoderButtonR ) && !( data.lastButtons & kNT_encoderButtonR ) )
			pThis->setupEditMode = !pThis->setupEditMode;

		if ( data.encoders[1] != 0 )
		{
			if ( pThis->setupEditMode )
			{
				int paramIndex = params[ pThis->setupSelectedItem ];
				int current = self->v[paramIndex];
				SetParam( pThis, paramIndex, current + data.encoders[1] );
			}
			else
			{
				pThis->setupSelectedItem += data.encoders[1];
				if ( pThis->setupSelectedItem < 0 ) pThis->setupSelectedItem = 0;
				if ( pThis->setupSelectedItem >= itemCount ) pThis->setupSelectedItem = itemCount - 1;
			}
		}
	}
	else
	{
		// Graph pages (Envelopes/LFOs) and bar pages (Model..FM) share one
		// control scheme. Physical control mapping (matches the hardware's
		// vertical layout, per user feedback): Pot L -> box 0, Pot C -> box 1,
		// Encoder R -> box 2 (3rd), Pot R -> box 3 (4th/last). Pots use
		// relative/incremental tracking (see PotRelativeUpdate()) since they
		// report absolute position and the currently-displayed page has no
		// relation to wherever a pot physically happens to be sitting;
		// Encoder R is a relative delta already and never has this problem.
		//
		// On the 9 concept bar pages (not Model), button 4 cycles a quick
		// modulation-depth-edit mode - Normal -> Env1 -> Env2 -> LFO1 ->
		// LFO2 -> Normal - so the same 4 controls edit that source's Mod
		// Matrix depth for each slot instead of the concept's base value,
		// without needing to leave the page for the Mod Matrix list (see
		// FindOrAllocRoute()).
		int concept = kPageConcept[ pThis->currentPage ];
		bool isBarPage = ( kPageType[pThis->currentPage] == kPageTypeBar );

		if ( isBarPage && concept != -1 && ( data.controls & kNT_button4 ) && !( data.lastButtons & kNT_button4 ) )
			pThis->barPageMode = ( pThis->barPageMode + 1 ) % kNumModSources;

		bool editingDepth = isBarPage && concept != -1 && pThis->barPageMode != kModSrcNone;
		const uint8_t* baseParams = kPageParams[ pThis->currentPage ];

		// Clicking the pot/encoder for a mapped slot while in quick-edit
		// mode opens a delete-confirm dialog (see deleteConfirmOpen's
		// comment) instead of adjusting the depth - only meaningful when
		// that slot actually has a route to delete.
		if ( editingDepth )
		{
			static const uint16_t kSlotButton[kNumSlots] = { kNT_potButtonL, kNT_potButtonC, kNT_encoderButtonR, kNT_potButtonR };
			for ( int slotIdx=0; slotIdx<kNumSlots; ++slotIdx )
			{
				if ( ( data.controls & kSlotButton[slotIdx] ) && !( data.lastButtons & kSlotButton[slotIdx] )
					&& FindRoute( pThis, pThis->barPageMode, slotIdx, concept ) >= 0 )
				{
					pThis->deleteConfirmOpen = true;
					pThis->deleteConfirmSlot = slotIdx;
					pThis->deleteConfirmChoice = 0;	// default No - never accidentally delete
				}
			}
		}

		if ( data.controls & kNT_potL )
			ApplyBarPot( pThis, 0, data.pots[0], editingDepth, pThis->barPageMode, concept, baseParams, 0 );
		if ( data.controls & kNT_potC )
			ApplyBarPot( pThis, 1, data.pots[1], editingDepth, pThis->barPageMode, concept, baseParams, 1 );
		if ( data.controls & kNT_potR )
			ApplyBarPot( pThis, 2, data.pots[2], editingDepth, pThis->barPageMode, concept, baseParams, 3 );
		if ( data.encoders[1] != 0 )
			ApplyBarEncoder( pThis, data.encoders[1], editingDepth, pThis->barPageMode, concept, baseParams, 2 );
	}
}

void	setupUi( _NT_algorithm* self, _NT_float3& pots )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;
	pThis->potHasLastPos[0] = pThis->potHasLastPos[1] = pThis->potHasLastPos[2] = false;
	pThis->potAccum[0] = pThis->potAccum[1] = pThis->potAccum[2] = 0.0f;
	if ( kPageType[pThis->currentPage] == kPageTypeList )
	{
		pots[0] = pots[1] = pots[2] = 0.0f;
		return;
	}
	// Pot R drives box 3 (params[3]), not box 2 - see customUi().
	const uint8_t* params = kPageParams[ pThis->currentPage ];
	static const int kPotToParamSlot[3] = { 0, 1, 3 };
	for ( int i=0; i<3; ++i )
	{
		const _NT_parameter& p = self->parameters[ params[ kPotToParamSlot[i] ] ];
		float range = (float)( p.max - p.min );
		pots[i] = range > 0.0f ? ( self->v[ params[ kPotToParamSlot[i] ] ] - p.min ) / range : 0.0f;
	}
}

// ---------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------

// Every custom-UI page shares one header: the page's title on the left, a
// divider line marking off the content area below, and (space permitting)
// a small "lnxdrum v5.0" watermark top-right - `rightTag` overrides that
// watermark for pages that need the top-right corner for something more
// important (DrawSetupPage's "EDIT" indicator), so the two never overlap.
// Deliberately compact (tiny text, divider at y=10) - an earlier version
// used the larger normal-text header used elsewhere, which ate into the
// content area below for no benefit (the page title doesn't need to be
// any more prominent than the content it's labelling).
constexpr int kHeaderDividerY = 10;
static void DrawHeader( const char* title, const char* rightTag = "lnxdrum v5.0" )
{
	NT_drawText( 4, 7, title, 15, kNT_textLeft, kNT_textTiny );
	if ( rightTag )
		NT_drawText( 252, 7, rightTag, 5, kNT_textRight, kNT_textTiny );
	NT_drawShapeI( kNT_line, 0, kHeaderDividerY, 255, kHeaderDividerY, 4 );
}

static void DrawSetupPage( _drumMachineAlgorithm* pThis, const char* title )
{
	DrawHeader( title, pThis->setupEditMode ? "EDIT" : "lnxdrum v5.0" );

	int itemCount = SetupPageItemCount( pThis->currentPage );
	const uint8_t* params = SetupPageParams( pThis->currentPage );

	// Scrolling window: keeps setupSelectedItem on screen regardless of
	// itemCount - needed once Envelopes (128 items) exists, since a fixed
	// first-N-items view would never show most of the list.
	constexpr int kVisibleItems = 5;
	int windowStart = pThis->setupSelectedItem - kVisibleItems / 2;
	if ( windowStart > itemCount - kVisibleItems ) windowStart = itemCount - kVisibleItems;
	if ( windowStart < 0 ) windowStart = 0;

	int y = kHeaderDividerY + 4;
	for ( int i=windowStart; i<itemCount && i<windowStart+kVisibleItems && y < 62; ++i, y += 8 )
	{
		int paramIndex = params[i];
		bool selected = ( i == pThis->setupSelectedItem );
		int colour = selected ? 15 : 8;
		NT_drawText( 4, y, pThis->parameters[paramIndex].name, colour, kNT_textLeft, kNT_textTiny );

		char buff[16];
		if ( pThis->parameters[paramIndex].unit == kNT_unitEnum && pThis->parameters[paramIndex].enumStrings != NULL )
		{
			int enumIdx = pThis->v[paramIndex] - pThis->parameters[paramIndex].min;
			NT_drawText( 160, y, pThis->parameters[paramIndex].enumStrings[enumIdx], colour, kNT_textLeft, kNT_textTiny );
		}
		else
		{
			NT_intToString( buff, pThis->v[paramIndex] );
			NT_drawText( 160, y, buff, colour, kNT_textLeft, kNT_textTiny );
		}
	}
}

// Samples `numSamples` points of `level(i/(numSamples-1))` (must return 0..1)
// across [x0,x1] and connects them with short line segments - the general
// curve-drawing primitive both the unipolar (AD/ADSR) and bipolar (LFO1/2)
// graphs below are built from.
template <typename LevelFn>
static void DrawCurve( int x0, int y0, int x1, int y1, int numSamples, int colour, LevelFn level )
{
	int prevX = x0;
	int prevY = y1 - (int)( level( 0.0f ) * ( y1 - y0 ) );
	for ( int i=1; i<numSamples; ++i )
	{
		float frac = (float)i / (float)( numSamples - 1 );
		int x = x0 + ( x1 - x0 ) * i / ( numSamples - 1 );
		int y = y1 - (int)( level( frac ) * ( y1 - y0 ) );
		NT_drawShapeI( kNT_line, prevX, prevY, x, y, colour );
		prevX = x; prevY = y;
	}
}

// Graph boxes are half-height (26px vs a bar page's 52px), leaving room
// below each for its name + current value - a HUD/scope-style readout
// rather than a full-height plot. Shared layout for both graph pages below.
static void DrawGraphBoxOutline( int bx0, int bx1, int y0, int y1 )
{
	// Dim outline + a faint scope-style grid (quarter ticks) - box brightness
	// is deliberately low so the bright curve line reads as the thing
	// actually lit up, like a scope trace.
	NT_drawShapeI( kNT_box, bx0, y0, bx1, y1, 4 );
	for ( int t=1; t<4; ++t )
	{
		int gx = bx0 + ( bx1 - bx0 ) * t / 4;
		NT_drawShapeI( kNT_line, gx, y0, gx, y1, 1 );
	}
	int midY = ( y0 + y1 ) / 2;
	NT_drawShapeI( kNT_line, bx0, midY, bx1, midY, 2 );
}

static void DrawEnvelopesPage( _drumMachineAlgorithm* pThis )
{
	DrawHeader( "ENVELOPES" );

	constexpr int y0 = 14;
	constexpr int y1 = 40;
	constexpr int boxWidth = 100;
	constexpr int gap = 20;
	constexpr int x0 = 14;
	constexpr int numSamples = 32;

	static const char* const kNames[2] = { "ENV1", "ENV2" };
	static const int kMorphParams[2] = { kParamEnv1Morph, kParamEnv2Morph };
	static const int kShapeParams[2] = { kParamEnv1Shape, kParamEnv2Shape };
	// Fixed preview pattern for the Random shape corner - not a live random
	// draw (there's no single "the" trigger to preview; each of the 4 slots
	// has its own live instance/randomSteps - see AdvanceEnvelope()), just
	// enough to make the stepped contour's character visible in the graph.
	static const float kPreviewRandomSteps[6] = { 0.6f, 0.3f, 0.8f, 0.2f, 0.5f, 0.9f };

	for ( int i=0; i<2; ++i )
	{
		int bx0 = x0 + i * ( boxWidth + gap );
		int bx1 = bx0 + boxWidth;
		DrawGraphBoxOutline( bx0, bx1, y0, y1 );

		int morphParam = pThis->v[ kMorphParams[i] ];
		int shapeParam = pThis->v[ kShapeParams[i] ];

		// Preview duration: longest of the AD and ADSR corners' total time at
		// this Morph setting, so the curve never gets cut off mid-contour
		// regardless of which corner(s) Shape is blending between.
		float attackS, decayS;
		ADShapeToTimes( morphParam, &attackS, &decayS );
		float totalS = attackS + decayS;
		float aS, dS, hS, rS, sL;
		ADSRShapeToTimes( morphParam, &aS, &dS, &hS, &rS, &sL );
		float adsrTotal = aS + dS + hS + rS;
		if ( adsrTotal > totalS ) totalS = adsrTotal;
		if ( totalS < 0.05f ) totalS = 0.05f;

		DrawCurve( bx0, y0, bx1, y1, numSamples, 15, [morphParam, shapeParam, totalS]( float frac )
		{
			bool finished;
			return BlendEnvelopeLevel( frac * totalS, morphParam, shapeParam, kPreviewRandomSteps, &finished );
		} );

		char valueBuff[16];
		NT_intToString( valueBuff, shapeParam );
		NT_drawText( bx0 + boxWidth/2, y1 + 8, kNames[i], 15, kNT_textCentre, kNT_textTiny );
		NT_drawText( bx0 + boxWidth/2, y1 + 16, valueBuff, 8, kNT_textCentre, kNT_textTiny );
	}
}

static void DrawLfosPage( _drumMachineAlgorithm* pThis )
{
	DrawHeader( "LFOS" );

	constexpr int y0 = 14;
	constexpr int y1 = 40;
	constexpr int boxWidth = 100;
	constexpr int gap = 20;
	constexpr int x0 = 14;
	constexpr int numSamples = 32;

	static const char* const kNames[2] = { "LFO1", "LFO2" };
	static const int kRateParams[2] = { kParamLfo1Rate, kParamLfo2Rate };
	static const int kShapeParams[2] = { kParamLfo1Shape, kParamLfo2Shape };

	for ( int i=0; i<2; ++i )
	{
		int bx0 = x0 + i * ( boxWidth + gap );
		int bx1 = bx0 + boxWidth;
		DrawGraphBoxOutline( bx0, bx1, y0, y1 );

		int shapeParam = pThis->v[ kShapeParams[i] ];
		DrawCurve( bx0, y0, bx1, y1, numSamples, 15, [shapeParam]( float frac )
		{
			return MorphedWave( frac, shapeParam ) * 0.5f + 0.5f;
		} );

		int rateIdx = pThis->v[ kRateParams[i] ];
		CONSTRAIN( rateIdx, 0, (int)ARRAY_SIZE(kEnumLfoRate) - 1 );
		NT_drawText( bx0 + boxWidth/2, y1 + 8, kNames[i], 15, kNT_textCentre, kNT_textTiny );
		NT_drawText( bx0 + boxWidth/2, y1 + 16, kEnumLfoRate[rateIdx], 8, kNT_textCentre, kNT_textTiny );
	}
}

static char const * const kQuickModeNames[kNumModSources] = {
	NULL, "ENV1 DEPTH", "ENV2 DEPTH", "LFO1 DEPTH", "LFO2 DEPTH",
};
static char const * const kModSourceTag[kNumModSources] = { NULL, "E1", "E2", "L1", "L2" };

static void DrawBarPage( _drumMachineAlgorithm* pThis )
{
	int page = pThis->currentPage;
	int concept = kPageConcept[page];
	bool editingDepth = ( concept != -1 ) && ( pThis->barPageMode != kModSrcNone );

	if ( editingDepth )
	{
		char titleBuff[32];
		strcpy( titleBuff, kPageNames[page] );
		strcat( titleBuff, " - " );
		strcat( titleBuff, kQuickModeNames[pThis->barPageMode] );
		DrawHeader( titleBuff );
	}
	else
	{
		DrawHeader( kPageNames[page] );
	}

	const uint8_t* params = kPageParams[page];
	bool bipolar = editingDepth ? true : kPageBipolar[page];

	// Equal-width bars, centered as a group on the 256px-wide display, now
	// reaching almost the full height below the (compact) header instead of
	// leaving a large unused band at the bottom. All the small labelling -
	// slot name, mapped-source tags - lives in a narrow column to the right
	// of each bar rather than a row underneath, so the reclaimed vertical
	// space goes to the bar itself, not wasted margin.
	constexpr int y0 = kHeaderDividerY + 4;
	constexpr int y1 = 60;
	constexpr int barWidth = 44;
	constexpr int labelColWidth = 18;
	constexpr int slotFootprint = barWidth + labelColWidth;
	constexpr int x0 = ( 256 - kNumSlots * slotFootprint ) / 2;

	for ( int slot=0; slot<kNumSlots; ++slot )
	{
		// In quick-edit mode, the bar shows the resolved route's depth
		// (read-only lookup - drawing must never allocate a route, only
		// actually turning a control does, via FindOrAllocRoute() in
		// customUi()); an unmapped slot just shows as 0/centered.
		int paramIndex = editingDepth ? -1 : params[slot];
		int route = editingDepth ? FindRoute( pThis, pThis->barPageMode, slot, concept ) : -1;
		int value = editingDepth ? ( route < 0 ? 0 : pThis->v[ ModRouteParam( route, kModParamDepth ) ] ) : pThis->v[paramIndex];
		int pMin = editingDepth ? -100 : pThis->parameters[paramIndex].min;
		int pMax = editingDepth ? 100 : pThis->parameters[paramIndex].max;

		int bx0 = x0 + slot * slotFootprint;
		int bx1 = bx0 + barWidth;

		NT_drawShapeI( kNT_box, bx0, y0, bx1, y1, 8 );

		float range = (float)( pMax - pMin );
		float t = range > 0.0f ? ( value - pMin ) / range : 0.0f;

		if ( bipolar )
		{
			// t in [0,1] maps to signedT in [-1,1]; fill from the center
			// line out towards whichever side signedT falls on.
			int midY = ( y0 + y1 ) / 2;
			float signedT = t * 2.0f - 1.0f;
			int extent = (int)( signedT * ( midY - y0 ) );
			int top = signedT >= 0.0f ? midY - extent : midY;
			int bottom = signedT >= 0.0f ? midY : midY - extent;
			NT_drawShapeI( kNT_rectangle, bx0 + 1, top, bx1 - 1, bottom, 12 );
			NT_drawShapeI( kNT_line, bx0, midY, bx1, midY, 15 );
		}
		else
		{
			int fillTop = y1 - (int)( t * ( y1 - y0 ) );
			NT_drawShapeI( kNT_rectangle, bx0 + 1, fillTop, bx1 - 1, y1 - 1, 12 );
		}

		auto valueToY = [&]( float v )
		{
			float vt = range > 0.0f ? ( v - pMin ) / range : 0.0f;
			CONSTRAIN( vt, 0.0f, 1.0f );
			if ( !bipolar )
				return y1 - (int)( vt * ( y1 - y0 ) );
			int midY = ( y0 + y1 ) / 2;
			float signedVt = vt * 2.0f - 1.0f;
			return midY - (int)( signedVt * ( midY - y0 ) );
		};

		// Modulation overlay (normal mode only - in quick-edit mode the bar
		// itself already *is* one route's depth): one thin contrasted line
		// per source currently mapped to this concept/slot - "a row of bars
		// if multiple apply" - at the value that source would push to at its
		// current depth, plus a brighter line at the actual live (combined,
		// all-sources) modulated value. Skipped on Model (concept == -1).
		if ( !editingDepth && concept != -1 )
		{
			for ( int s=kModSrcEnv1; s<=kModSrcLfo2; ++s )
			{
				int r = FindRoute( pThis, s, slot, concept );
				if ( r < 0 )
					continue;
				int depth = pThis->v[ ModRouteParam( r, kModParamDepth ) ];
				int yy = valueToY( (float)value + ( depth * 0.01f ) * range );
				NT_drawShapeI( kNT_line, bx0 + 2, yy, bx1 - 2, yy, 7 );
			}

			float modulated = ModulatedViaMatrix( pThis, concept, slot, (float)value, pThis->currentEnv1Level[slot], pThis->currentEnv2Level[slot] );
			int modY = valueToY( modulated );
			NT_drawShapeI( kNT_line, bx0, modY, bx1, modY, 15 );
		}

		// Velocity: a small filled square "dot" at the top of the label
		// column (right of the bar), brightness 0 (no recent hit) to 15
		// (just hit) tracking velocityMeter[slot]'s decay (see
		// DecayVelocityMeters()) - visually separates "incoming note energy"
		// (this dot, top) from "what's mapped" (the label stack below it,
		// bottom-up) and "how modulation applies right now" (lines inside
		// the bar), instead of one combined pumping bar. A filled rectangle
		// at 1px "radius" (a crisp 3x3 square) rather than kNT_circle - an
		// unfilled circle (the only circle variant available) reads as a
		// much bigger blob than its bounding box suggests at this size.
		{
			int dotR = 1;
			int dotCx = bx1 + 2 + dotR;	// same 2px gap from the bar as the label text below
			int dotCy = y0 + 3;
			int vcol = (int)( pThis->velocityMeter[slot] * 15.0f );
			CONSTRAIN( vcol, 0, 15 );
			NT_drawShapeI( kNT_rectangle, dotCx - dotR, dotCy - dotR, dotCx + dotR, dotCy + dotR, vcol );
		}

		char buff[16];
		// An unmapped slot in quick-edit mode with no free route anywhere
		// can't be assigned by turning its control - say so explicitly
		// instead of just showing an inert "0" (see AnyRouteFree()).
		if ( editingDepth && route < 0 && !AnyRouteFree( pThis ) )
		{
			NT_drawText( bx0 + barWidth/2, ( y0 + y1 ) / 2 + 3, "FULL", 8, kNT_textCentre, kNT_textTiny );
		}
		else if ( !editingDepth && pThis->parameters[paramIndex].unit == kNT_unitEnum && pThis->parameters[paramIndex].enumStrings != NULL )
		{
			int idx = value - pMin;
			if ( idx >= 0 && idx <= ( pMax - pMin ) )
				NT_drawText( bx0 + barWidth/2, ( y0 + y1 ) / 2 + 3, pThis->parameters[paramIndex].enumStrings[idx], 15, kNT_textCentre, kNT_textTiny );
		}
		else if ( !editingDepth && pThis->parameters[paramIndex].unit == kNT_unitHasStrings )
		{
			// Sample page - dynamic file name (or "None"), not a static
			// enumStrings list, since the file list is only known at
			// runtime - see GetSampleDisplayName().
			char sbuff[32];
			if ( GetSampleDisplayName( pThis, slot, value, sbuff, sizeof(sbuff) ) )
				NT_drawText( bx0 + barWidth/2, ( y0 + y1 ) / 2 + 3, sbuff, 15, kNT_textCentre, kNT_textTiny );
		}
		else
		{
			NT_intToString( buff, value );
			NT_drawText( bx0 + barWidth/2, ( y0 + y1 ) / 2 + 3, buff, 15, kNT_textCentre, kNT_textTiny );
		}

		// Labelling column, right of the bar: a vertical stack, left-aligned,
		// bottom-anchored to the bar's own bottom line - the slot name's
		// baseline sits exactly on y1 (matching how the inline value text
		// above is baseline-positioned via "+3" off the vertical center, not
		// top-anchored), and each mapped source ("E1"/"E2"/"L1"/"L2") stacks
		// one row above the last as more mappings apply, instead of a row of
		// text underneath the bar (which left a large unused band at the
		// bottom of the screen).
		int labelX = bx1 + 2;
		int labelY = y1;
		NT_drawText( labelX, labelY, kSlotNames[slot], 15, kNT_textLeft, kNT_textTiny );

		if ( !editingDepth && concept != -1 )
		{
			for ( int s=kModSrcEnv1; s<=kModSrcLfo2; ++s )
			{
				if ( FindRoute( pThis, s, slot, concept ) < 0 )
					continue;
				labelY -= 8;
				NT_drawText( labelX, labelY, kModSourceTag[s], 10, kNT_textLeft, kNT_textTiny );
			}
		}
	}
}

static void DrawPresetMenu( _drumMachineAlgorithm* pThis )
{
	NT_drawText( 4, 7, "PRESET", 15, kNT_textLeft, kNT_textTiny );
	if ( pThis->presetMenuStage == 0 )
		NT_drawText( 251, 7, "PRESS", 8, kNT_textRight, kNT_textTiny );
	else
	{
		NT_drawText( 197, 7, "LOAD", pThis->presetMenuAction == 0 ? 15 : 5, kNT_textLeft, kNT_textTiny );
		NT_drawText( 232, 7, "SAVE", pThis->presetMenuAction == 1 ? 15 : 5, kNT_textLeft, kNT_textTiny );
	}
	NT_drawShapeI( kNT_line, 0, kHeaderDividerY, 255, kHeaderDividerY, 4 );

	// Scrolling window - kNumPresets (64) doesn't fit in the ~5 visible rows,
	// so without this the selection could scroll right off screen with no
	// indication (matches the same fix already applied to DrawSetupPage()).
	constexpr int kVisibleItems = 5;
	int windowStart = pThis->presetMenuIndex - kVisibleItems / 2;
	if ( windowStart > kNumPresets - kVisibleItems ) windowStart = kNumPresets - kVisibleItems;
	if ( windowStart < 0 ) windowStart = 0;

	int y = kHeaderDividerY + 4;
	for ( int i=windowStart; i<kNumPresets && i<windowStart+kVisibleItems && y < 62; ++i, y += 8 )
	{
		bool selected = ( i == pThis->presetMenuIndex );
		if ( selected )
			NT_drawShapeI( kNT_rectangle, 2, y - 1, 253, y + 7, 3 );

		char label[32];
		char num[4];
		NT_intToString( num, i + 1 );
		strcpy( label, num );
		strcat( label, " " );
		strcat( label, pThis->presetBankValid[i] ? pThis->presetName[i] : "(empty)" );
		NT_drawText( 4, y, label, selected ? 15 : 8, kNT_textLeft, kNT_textTiny );
	}
}

static void DrawDeleteConfirm( _drumMachineAlgorithm* pThis )
{
	DrawHeader( "DELETE MODIFIER?", NULL );
	NT_drawText( 100, 24, "NO", pThis->deleteConfirmChoice == 0 ? 15 : 5, kNT_textCentre, kNT_textNormal );
	NT_drawText( 156, 24, "YES", pThis->deleteConfirmChoice == 1 ? 15 : 5, kNT_textCentre, kNT_textNormal );

	int concept = kPageConcept[ pThis->currentPage ];
	int route = FindRoute( pThis, pThis->barPageMode, pThis->deleteConfirmSlot, concept );
	char msg[48] = "";
	strcpy( msg, kSlotNames[ pThis->deleteConfirmSlot ] );
	strcat( msg, " " );
	strcat( msg, kQuickModeNames[ pThis->barPageMode ] );
	if ( route >= 0 )
	{
		char depthBuff[8];
		NT_intToString( depthBuff, pThis->v[ ModRouteParam( route, kModParamDepth ) ] );
		strcat( msg, " (" );
		strcat( msg, depthBuff );
		strcat( msg, ")" );
	}
	NT_drawText( 128, 36, msg, 8, kNT_textCentre, kNT_textTiny );
}

bool	draw( _NT_algorithm* self )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;

	if ( pThis->presetMenuOpen )
		DrawPresetMenu( pThis );
	else if ( pThis->deleteConfirmOpen )
		DrawDeleteConfirm( pThis );
	else if ( kPageType[pThis->currentPage] == kPageTypeList )
		DrawSetupPage( pThis, kPageNames[pThis->currentPage] );
	else if ( pThis->currentPage == kPageEnvelopes )
		DrawEnvelopesPage( pThis );
	else if ( pThis->currentPage == kPageLfos )
		DrawLfosPage( pThis );
	else
		DrawBarPage( pThis );

	return true;
}

// ---------------------------------------------------------------------
// Preset bank persistence - part of whichever main disting NT preset this
// algorithm instance is in (forks on "Save As", empty for a brand new
// instance); the plugin API has no file-write capability for algorithms to
// use to maintain a library independent of that.
// ---------------------------------------------------------------------

void	serialise( _NT_algorithm* self, _NT_jsonStream& stream )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;
	stream.addMemberName( "presets" );
	stream.openArray();
	for ( int slot=0; slot<kNumPresets; ++slot )
	{
		stream.openObject();
		stream.addMemberName( "valid" );
		stream.addBoolean( pThis->presetBankValid[slot] );
		// An empty/never-saved slot has nothing worth writing - the "v"/
		// "mod" arrays would just be every param's own default, 116 numbers
		// of pure padding, x however many of the 64 slots are still unused
		// (most of them, for most users, most of the time). Writing that
		// unconditionally for every slot regardless of use is what actually
		// caused a real "preset too complex to load, please report as a
		// bug" failure once the Sample-layer/FM Mode params were added to
		// this per-slot payload - confirmed by a direct test (a freshly
		// re-saved preset failed to load; an older, smaller-payload one
		// kept loading fine even against this same build). Skipping unused
		// slots' payload entirely scales with how much of the bank is
		// actually in use instead of always paying for the full 64, which
		// should buy back much more headroom than removing any one
		// feature's params would - deserialise() already tolerates a slot
		// object with "name"/"v"/"mod" absent (its defaults kick in), so no
		// changes needed on that side.
		if ( pThis->presetBankValid[slot] )
		{
			stream.addMemberName( "name" );
			stream.addString( pThis->presetName[slot] );
			stream.addMemberName( "v" );
			stream.openArray();
			for ( int i=0; i<kNumKitParams; ++i )
				stream.addNumber( (int)pThis->presetBank[slot][i] );
			stream.closeArray();
			stream.addMemberName( "mod" );
			stream.openArray();
			for ( int i=0; i<kNumModParams; ++i )
				stream.addNumber( (int)pThis->modBank[slot][i] );
			stream.closeArray();
		}
		stream.closeObject();
	}
	stream.closeArray();
}

bool	deserialise( _NT_algorithm* self, _NT_jsonParse& parse )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;
	// The host calls this while restoring a saved preset - which itself
	// involves reading the preset file from the same SD card the sample
	// layer uses. See sampleSystemGraceSamples's comment: overlapping that
	// with the sample-streaming API caused a real device freeze, confirmed
	// specifically tied to loading while actively playing (not idle).
	ExtendSampleSystemGrace( pThis );
	int numMembers;
	if ( !parse.numberOfObjectMembers( numMembers ) )
		return false;

	for ( int m=0; m<numMembers; ++m )
	{
		if ( parse.matchName( "presets" ) )
		{
			int numSlotsInFile;
			if ( !parse.numberOfArrayElements( numSlotsInFile ) )
				return false;
			for ( int slot=0; slot<numSlotsInFile; ++slot )
			{
				bool valid = false;
				bool nameFound = false;
				bool modFound = false;
				char name[kPresetNameLen];
				name[0] = '\0';
				int values[kNumKitParams];
				memset( values, 0, sizeof(values) );
				int modValues[kNumModParams];
				memset( modValues, 0, sizeof(modValues) );

				int numFields;
				if ( !parse.numberOfObjectMembers( numFields ) )
					return false;
				for ( int f=0; f<numFields; ++f )
				{
					if ( parse.matchName( "valid" ) )
					{
						if ( !parse.boolean( valid ) ) return false;
					}
					else if ( parse.matchName( "name" ) )
					{
						const char* str;
						if ( !parse.string( str ) ) return false;
						strncpy( name, str, kPresetNameLen - 1 );
						name[kPresetNameLen - 1] = '\0';
						nameFound = true;
					}
					else if ( parse.matchName( "v" ) )
					{
						int numVals;
						if ( !parse.numberOfArrayElements( numVals ) )
							return false;
						for ( int i=0; i<numVals; ++i )
						{
							int val = 0;
							if ( !parse.number( val ) )
								return false;
							if ( i < kNumKitParams )
								values[i] = val;
						}
					}
					else if ( parse.matchName( "mod" ) )
					{
						int numVals;
						if ( !parse.numberOfArrayElements( numVals ) )
							return false;
						for ( int i=0; i<numVals; ++i )
						{
							int val = 0;
							if ( !parse.number( val ) )
								return false;
							if ( i < kNumModParams )
								modValues[i] = val;
						}
						modFound = true;
					}
					else if ( !parse.skipMember() )
						return false;
				}

				if ( slot < kNumPresets )
				{
					pThis->presetBankValid[slot] = valid;
					// Presets saved before auto-naming/envelopes/FM existed
					// won't have "name"/"mod" fields - default whatever's
					// missing rather than leaving the slot blank or loading
					// garbage (modBank already holds each parameter's own
					// default from construct() if "mod" is absent here).
					if ( nameFound )
						strcpy( pThis->presetName[slot], name );
					else
						GenerateRandomPresetName( pThis->presetName[slot] );
					for ( int i=0; i<kNumKitParams; ++i )
						pThis->presetBank[slot][i] = (int16_t)values[i];
					if ( modFound )
						for ( int i=0; i<kNumModParams; ++i )
							pThis->modBank[slot][i] = (int16_t)modValues[i];
				}
			}
		}
		else if ( !parse.skipMember() )
			return false;
	}
	return true;
}

static const _NT_factory factory =
{
	.guid = NT_MULTICHAR( 'D', 'r', 'm', 'M' ),
	.name = "Drum Machine",
	.description = "4-slot drum machine (bass/snare/closed hat/open hat)",
	.numSpecifications = 0,
	.calculateRequirements = calculateRequirements,
	.construct = construct,
	.parameterChanged = parameterChanged,
	.step = step,
	.draw = draw,
	.midiRealtime = midiRealtime,
	.midiMessage = midiMessage,
	.tags = kNT_tagInstrument,
	.hasCustomUi = hasCustomUi,
	.customUi = customUi,
	.setupUi = setupUi,
	.serialise = serialise,
	.deserialise = deserialise,
	.parameterString = parameterString,
};

uintptr_t pluginEntry( _NT_selector selector, uint32_t data )
{
	switch ( selector )
	{
	case kNT_selector_version:
		return kNT_apiVersionCurrent;
	case kNT_selector_numFactories:
		return 1;
	case kNT_selector_factoryInfo:
		return (uintptr_t)( ( data == 0 ) ? &factory : NULL );
	}
	return 0;
}
