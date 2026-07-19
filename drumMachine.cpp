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

// Out-of-line definitions for the vendored code's extern/static members.
// Single-translation-unit build (each plugin .cpp compiles standalone), so
// including units.cc's LUT data directly here is safe - no ODR risk.
float plaits::kSampleRate = 48000.0f;
uint32_t stmlib::Random::rng_state_ = 1;
#include "third_party/mi_drums/stmlib/dsp/units.cc"

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

// ---------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------

enum
{
	kSlotBD, kSlotSD, kSlotCH, kSlotOH,
	kNumSlots,
};

enum
{
	// Page 0: Routing
	kParamOutBD, kParamOutBDMode,
	kParamOutSD, kParamOutSDMode,
	kParamOutCH, kParamOutCHMode,
	kParamOutOH, kParamOutOHMode,

	// Page 1: MIDI
	kParamMidiMode,
	kParamMidiChannel,
	kParamMidiChBD, kParamMidiChSD, kParamMidiChCH, kParamMidiChOH,
	kParamMidiNoteBD, kParamMidiNoteSD, kParamMidiNoteCH, kParamMidiNoteOH,

	// Page 2: Model
	kParamModelBD, kParamModelSD, kParamModelCH, kParamModelOH,
	// Page 3: Release
	kParamRelBD, kParamRelSD, kParamRelCH, kParamRelOH,
	// Page 4: Filter (DJ-style bipolar: <0 lowpass, >0 highpass, 0 = bypass)
	kParamFiltBD, kParamFiltSD, kParamFiltCH, kParamFiltOH,
	// Page 5: Waveshaper
	kParamDriveBD, kParamDriveSD, kParamDriveCH, kParamDriveOH,
	// Page 6: Pitch
	kParamPitchBD, kParamPitchSD, kParamPitchCH, kParamPitchOH,
	// Page 7: Volume
	kParamVolBD, kParamVolSD, kParamVolCH, kParamVolOH,
	// Page 8: Tone
	kParamToneBD, kParamToneSD, kParamToneCH, kParamToneOH,
	// Page 9: Character
	kParamCharBD, kParamCharSD, kParamCharCH, kParamCharOH,

	kNumParams,
};

static char const * const kEnumModelBD[] = { "Analog", "Synthetic" };
static char const * const kEnumModelSD[] = { "Analog", "Synthetic" };
static char const * const kEnumModelHat[] = { "808" };
static char const * const kEnumMidiMode[] = { "Note per slot", "Channel per slot" };

static const _NT_parameter parameters[] = {
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "BD Out", 1, 13 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "SD Out", 1, 14 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "CH Out", 1, 15 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "OH Out", 1, 16 )

	{ .name = "MIDI mode", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumMidiMode },
	{ .name = "MIDI channel", .min = 0, .max = 16, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "BD channel", .min = 0, .max = 16, .def = 1, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD channel", .min = 0, .max = 16, .def = 2, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH channel", .min = 0, .max = 16, .def = 3, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH channel", .min = 0, .max = 16, .def = 4, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "BD note", .min = 0, .max = 127, .def = 36, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD note", .min = 0, .max = 127, .def = 38, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH note", .min = 0, .max = 127, .def = 42, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH note", .min = 0, .max = 127, .def = 46, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD model", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumModelBD },
	{ .name = "SD model", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumModelSD },
	{ .name = "CH model", .min = 0, .max = 0, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumModelHat },
	{ .name = "OH model", .min = 0, .max = 0, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = kEnumModelHat },

	{ .name = "BD release", .min = 0, .max = 100, .def = 40, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD release", .min = 0, .max = 100, .def = 40, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH release", .min = 0, .max = 100, .def = 20, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH release", .min = 0, .max = 100, .def = 60, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD filter", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD filter", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH filter", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH filter", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD drive", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD drive", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH drive", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH drive", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD pitch", .min = 0, .max = 127, .def = 36, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD pitch", .min = 0, .max = 127, .def = 38, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH pitch", .min = 0, .max = 127, .def = 42, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH pitch", .min = 0, .max = 127, .def = 46, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD volume", .min = 0, .max = 100, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD volume", .min = 0, .max = 100, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH volume", .min = 0, .max = 100, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH volume", .min = 0, .max = 100, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD tone", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD tone", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH tone", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH tone", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "BD character", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "SD character", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "CH character", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "OH character", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
};

static const uint8_t pageRouting[] = { kParamOutBD, kParamOutBDMode, kParamOutSD, kParamOutSDMode, kParamOutCH, kParamOutCHMode, kParamOutOH, kParamOutOHMode };
static const uint8_t pageMidi[]    = { kParamMidiMode, kParamMidiChannel, kParamMidiChBD, kParamMidiChSD, kParamMidiChCH, kParamMidiChOH, kParamMidiNoteBD, kParamMidiNoteSD, kParamMidiNoteCH, kParamMidiNoteOH };
static const uint8_t pageModel[]   = { kParamModelBD, kParamModelSD, kParamModelCH, kParamModelOH };
static const uint8_t pageRelease[] = { kParamRelBD, kParamRelSD, kParamRelCH, kParamRelOH };
static const uint8_t pageFilter[]  = { kParamFiltBD, kParamFiltSD, kParamFiltCH, kParamFiltOH };
static const uint8_t pageDrive[]   = { kParamDriveBD, kParamDriveSD, kParamDriveCH, kParamDriveOH };
static const uint8_t pagePitch[]   = { kParamPitchBD, kParamPitchSD, kParamPitchCH, kParamPitchOH };
static const uint8_t pageVolume[]  = { kParamVolBD, kParamVolSD, kParamVolCH, kParamVolOH };
static const uint8_t pageTone[]    = { kParamToneBD, kParamToneSD, kParamToneCH, kParamToneOH };
static const uint8_t pageChar[]    = { kParamCharBD, kParamCharSD, kParamCharCH, kParamCharOH };

static const _NT_parameterPage pages[] = {
	{ .name = "Routing", .numParams = ARRAY_SIZE(pageRouting), .params = pageRouting },
	{ .name = "MIDI", .numParams = ARRAY_SIZE(pageMidi), .params = pageMidi },
	{ .name = "Model", .numParams = ARRAY_SIZE(pageModel), .params = pageModel },
	{ .name = "Release", .numParams = ARRAY_SIZE(pageRelease), .params = pageRelease },
	{ .name = "Filter", .numParams = ARRAY_SIZE(pageFilter), .params = pageFilter },
	{ .name = "Waveshaper", .numParams = ARRAY_SIZE(pageDrive), .params = pageDrive },
	{ .name = "Pitch", .numParams = ARRAY_SIZE(pagePitch), .params = pagePitch },
	{ .name = "Volume", .numParams = ARRAY_SIZE(pageVolume), .params = pageVolume },
	{ .name = "Tone", .numParams = ARRAY_SIZE(pageTone), .params = pageTone },
	{ .name = "Character", .numParams = ARRAY_SIZE(pageChar), .params = pageChar },
};

static const _NT_parameterPages parameterPages = {
	.numPages = ARRAY_SIZE(pages),
	.pages = pages,
};

// Bar-graph ("performance") pages: index 0 here == currentPage 2 (Model).
enum { kNumBarPages = 8, kNumSetupPages = 2, kNumPages = kNumSetupPages + kNumBarPages };

static const uint8_t* const kBarPageParams[kNumBarPages] = {
	pageModel, pageRelease, pageFilter, pageDrive, pagePitch, pageVolume, pageTone, pageChar,
};
static const char* const kBarPageNames[kNumBarPages] = {
	"MODEL", "RELEASE", "FILTER", "WAVESHAPE", "PITCH", "VOLUME", "TONE", "CHARACTER",
};
static const bool kBarPageBipolar[kNumBarPages] = {
	false, false, true, false, false, false, false, false,
};

static const char* const kSlotNames[kNumSlots] = { "BD", "SD", "CH", "OH" };

// ---------------------------------------------------------------------
// Voice DSP state
// ---------------------------------------------------------------------

struct _drumVoicePost
{
	stmlib::Svf lpFilter;
	stmlib::Svf hpFilter;
	plaits::Overdrive drive;
	int samplesUntilIdle;
	bool pendingTrigger;
	float pendingAccent;

	void Init()
	{
		lpFilter.Init();
		hpFilter.Init();
		drive.Init();
		samplesUntilIdle = 0;
		pendingTrigger = false;
		pendingAccent = 1.0f;
	}
};

struct _kickVoice : _drumVoicePost
{
	plaits::AnalogBassDrum analog;
	plaits::SyntheticBassDrum synthetic;

	void Init()
	{
		_drumVoicePost::Init();
		analog.Init();
		synthetic.Init();
	}
};

struct _snareVoice : _drumVoicePost
{
	plaits::AnalogSnareDrum analog;
	plaits::SyntheticSnareDrum synthetic;

	void Init()
	{
		_drumVoicePost::Init();
		analog.Init();
		synthetic.Init();
	}
};

struct _hatVoice : _drumVoicePost
{
	plaits::HiHat<plaits::SquareNoise, plaits::SwingVCA, true, false> hihat;
	float* scratch1;	// [maxFramesPerStep], points into dram
	float* scratch2;	// [maxFramesPerStep], points into dram

	void Init()
	{
		_drumVoicePost::Init();
		hihat.Init();
	}
};

struct _drumMachineAlgorithm_DTC
{
	_kickVoice kick;
	_snareVoice snare;
	_hatVoice closedHat;
	_hatVoice openHat;
};

struct _drumMachineAlgorithm : public _NT_algorithm
{
	_drumMachineAlgorithm( _drumMachineAlgorithm_DTC* dtc_ ) : dtc( dtc_ ) {}
	~_drumMachineAlgorithm() {}

	_drumMachineAlgorithm_DTC* dtc;
	float* renderScratch;	// [maxFramesPerStep], points into dram

	// Custom UI state - the standard page system can't tell draw() which
	// page is selected, so page position and value-editing are handled
	// entirely here instead of via the standard pot/encoder wiring.
	int currentPage;		// 0=Routing, 1=MIDI, 2..9 = bar-graph pages
	int setupSelectedItem;	// which item is "focused" on Routing/MIDI pages
};

// ---------------------------------------------------------------------
// Requirements / construction
// ---------------------------------------------------------------------

void	calculateRequirements( _NT_algorithmRequirements& req, const int32_t* specifications )
{
	req.numParameters = ARRAY_SIZE(parameters);
	req.sram = sizeof(_drumMachineAlgorithm);
	req.dtc = sizeof(_drumMachineAlgorithm_DTC);
	// renderScratch + 2x hat scratch buffers, sized to the largest block
	// this instance will ever be asked to render in one step() call.
	req.dram = 3 * NT_globals.maxFramesPerStep * sizeof(float);
	req.itc = 0;
}

_NT_algorithm*	construct( const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req, const int32_t* specifications )
{
	_drumMachineAlgorithm* alg = new (ptrs.sram) _drumMachineAlgorithm( (_drumMachineAlgorithm_DTC*)ptrs.dtc );

	plaits::kSampleRate = (float)NT_globals.sampleRate;

	float* dram = (float*)ptrs.dram;
	int maxFrames = NT_globals.maxFramesPerStep;
	alg->renderScratch = dram; dram += maxFrames;
	alg->dtc->closedHat.scratch1 = dram; dram += maxFrames;
	alg->dtc->closedHat.scratch2 = dram; dram += maxFrames;
	// openHat reuses closedHat's scratch1/scratch2 range is NOT safe (both
	// could be simultaneously "active"), so give it its own - re-derive
	// req.dram sizing note: this needs 4 scratch buffers total, not 3.
	alg->dtc->openHat.scratch1 = dram; dram += maxFrames;
	alg->dtc->openHat.scratch2 = dram; dram += maxFrames;

	alg->dtc->kick.Init();
	alg->dtc->snare.Init();
	alg->dtc->closedHat.Init();
	alg->dtc->openHat.Init();

	alg->currentPage = 0;
	alg->setupSelectedItem = 0;

	alg->parameters = parameters;
	alg->parameterPages = &parameterPages;
	return alg;
}

void	parameterChanged( _NT_algorithm* self, int p )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;
	if ( p == kParamMidiMode )
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

// ---------------------------------------------------------------------
// Audio rendering
// ---------------------------------------------------------------------

// Fixed generous idle tail: simpler and safer than trying to derive an
// exact tail length from the release parameter, and idle-skipping still
// pays off since most slots sit silent between hits most of the time.
static int IdleSamples()
{
	return (int)( plaits::kSampleRate * 2.0f );
}

// DJ-style bipolar filter (center = bypass, down = lowpass darkening, up =
// highpass thinning) + waveshaper + output level trim, applied in place.
static void ApplyPost( _drumVoicePost& v, float* buf, int numFrames, int filterParam, int driveParam, int volParam )
{
	if ( filterParam < 0 )
	{
		float t = ( -filterParam ) * 0.01f;
		float tt = t * t;	// quadratic taper - avoids needing a pow() call
		float cutoffHz = 18000.0f + tt * ( 80.0f - 18000.0f );
		float cutoffNorm = cutoffHz / plaits::kSampleRate;
		CONSTRAIN( cutoffNorm, 0.001f, 0.49f );
		v.lpFilter.set_f_q<stmlib::FREQUENCY_FAST>( cutoffNorm, 0.6f );
		v.lpFilter.Process<stmlib::FILTER_MODE_LOW_PASS>( buf, buf, numFrames );
	}
	else if ( filterParam > 0 )
	{
		float t = filterParam * 0.01f;
		float tt = t * t;
		float cutoffHz = 20.0f + tt * ( 8000.0f - 20.0f );
		float cutoffNorm = cutoffHz / plaits::kSampleRate;
		CONSTRAIN( cutoffNorm, 0.001f, 0.49f );
		v.hpFilter.set_f_q<stmlib::FREQUENCY_FAST>( cutoffNorm, 0.6f );
		v.hpFilter.Process<stmlib::FILTER_MODE_HIGH_PASS>( buf, buf, numFrames );
	}

	if ( driveParam > 0 )
		v.drive.Process( driveParam * 0.01f, buf, numFrames );

	float vol = volParam * 0.01f;
	for ( int i=0; i<numFrames; ++i )
		buf[i] *= vol;
}

static void ProcessKick( _drumMachineAlgorithm* pThis, float* busFrames, int numFrames )
{
	_kickVoice& v = pThis->dtc->kick;
	bool trig = v.pendingTrigger;
	v.pendingTrigger = false;
	if ( trig )
		v.samplesUntilIdle = IdleSamples();

	float* out = busFrames + ( pThis->v[kParamOutBD] - 1 ) * numFrames;
	bool replace = pThis->v[kParamOutBDMode];

	if ( v.samplesUntilIdle <= 0 && !trig )
	{
		if ( replace ) memset( out, 0, numFrames * sizeof(float) );
		return;
	}

	float* scratch = pThis->renderScratch;
	float f0 = kMidiNoteFreq[ pThis->v[kParamPitchBD] ] / plaits::kSampleRate;
	float decay = pThis->v[kParamRelBD] * 0.01f;
	float tone = pThis->v[kParamToneBD] * 0.01f;
	float character = pThis->v[kParamCharBD] * 0.01f;
	float accent = v.pendingAccent;

	if ( pThis->v[kParamModelBD] == 0 )
	{
		float attackFm = std::min( character * 2.0f, 1.0f );
		float selfFm = std::max( character * 2.0f - 1.0f, 0.0f );
		v.analog.Render( trig, accent, f0, tone, decay, attackFm, selfFm, scratch, numFrames );
	}
	else
	{
		v.synthetic.Render( false, trig, accent, f0, tone, decay, character, 0.5f, 0.5f, scratch, numFrames );
	}

	ApplyPost( v, scratch, numFrames, pThis->v[kParamFiltBD], pThis->v[kParamDriveBD], pThis->v[kParamVolBD] );

	v.samplesUntilIdle -= numFrames;
	if ( v.samplesUntilIdle < 0 ) v.samplesUntilIdle = 0;

	if ( replace )
		memcpy( out, scratch, numFrames * sizeof(float) );
	else
		for ( int i=0; i<numFrames; ++i ) out[i] += scratch[i];
}

static void ProcessSnare( _drumMachineAlgorithm* pThis, float* busFrames, int numFrames )
{
	_snareVoice& v = pThis->dtc->snare;
	bool trig = v.pendingTrigger;
	v.pendingTrigger = false;
	if ( trig )
		v.samplesUntilIdle = IdleSamples();

	float* out = busFrames + ( pThis->v[kParamOutSD] - 1 ) * numFrames;
	bool replace = pThis->v[kParamOutSDMode];

	if ( v.samplesUntilIdle <= 0 && !trig )
	{
		if ( replace ) memset( out, 0, numFrames * sizeof(float) );
		return;
	}

	float* scratch = pThis->renderScratch;
	float f0 = kMidiNoteFreq[ pThis->v[kParamPitchSD] ] / plaits::kSampleRate;
	float decay = pThis->v[kParamRelSD] * 0.01f;
	float tone = pThis->v[kParamToneSD] * 0.01f;
	float character = pThis->v[kParamCharSD] * 0.01f;
	float accent = v.pendingAccent;

	if ( pThis->v[kParamModelSD] == 0 )
		v.analog.Render( trig, accent, f0, tone, decay, character, scratch, numFrames );
	else
		v.synthetic.Render( false, trig, accent, f0, tone, decay, character, scratch, numFrames );

	ApplyPost( v, scratch, numFrames, pThis->v[kParamFiltSD], pThis->v[kParamDriveSD], pThis->v[kParamVolSD] );

	v.samplesUntilIdle -= numFrames;
	if ( v.samplesUntilIdle < 0 ) v.samplesUntilIdle = 0;

	if ( replace )
		memcpy( out, scratch, numFrames * sizeof(float) );
	else
		for ( int i=0; i<numFrames; ++i ) out[i] += scratch[i];
}

static void ProcessHat( _drumMachineAlgorithm* pThis, float* busFrames, int numFrames, bool isOpen )
{
	_hatVoice& v = isOpen ? pThis->dtc->openHat : pThis->dtc->closedHat;
	int outParam = isOpen ? kParamOutOH : kParamOutCH;
	int modeParam = isOpen ? kParamOutOHMode : kParamOutCHMode;
	int pitchParam = isOpen ? kParamPitchOH : kParamPitchCH;
	int relParam = isOpen ? kParamRelOH : kParamRelCH;
	int toneParam = isOpen ? kParamToneOH : kParamToneCH;
	int charParam = isOpen ? kParamCharOH : kParamCharCH;
	int filtParam = isOpen ? kParamFiltOH : kParamFiltCH;
	int driveParam = isOpen ? kParamDriveOH : kParamDriveCH;
	int volParam = isOpen ? kParamVolOH : kParamVolCH;

	bool trig = v.pendingTrigger;
	v.pendingTrigger = false;
	if ( trig )
		v.samplesUntilIdle = IdleSamples();

	float* out = busFrames + ( pThis->v[outParam] - 1 ) * numFrames;
	bool replace = pThis->v[modeParam];

	if ( v.samplesUntilIdle <= 0 && !trig )
	{
		if ( replace ) memset( out, 0, numFrames * sizeof(float) );
		return;
	}

	float* scratch = pThis->renderScratch;
	float f0 = kMidiNoteFreq[ pThis->v[pitchParam] ] / plaits::kSampleRate;
	float decay = pThis->v[relParam] * 0.01f;
	float tone = pThis->v[toneParam] * 0.01f;
	float noisiness = pThis->v[charParam] * 0.01f;
	float accent = v.pendingAccent;

	v.hihat.Render( trig, accent, f0, tone, decay, noisiness, v.scratch1, v.scratch2, scratch, numFrames );

	ApplyPost( v, scratch, numFrames, pThis->v[filtParam], pThis->v[driveParam], pThis->v[volParam] );

	v.samplesUntilIdle -= numFrames;
	if ( v.samplesUntilIdle < 0 ) v.samplesUntilIdle = 0;

	if ( replace )
		memcpy( out, scratch, numFrames * sizeof(float) );
	else
		for ( int i=0; i<numFrames; ++i ) out[i] += scratch[i];
}

void 	step( _NT_algorithm* self, float* busFrames, int numFramesBy4 )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;
	int numFrames = numFramesBy4 * 4;

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
	return kNT_encoderL | kNT_encoderR | kNT_encoderButtonR | kNT_potL | kNT_potC | kNT_potR;
}

static int SetupPageItemCount( int page )
{
	return page == 0 ? ARRAY_SIZE(pageRouting) : ARRAY_SIZE(pageMidi);
}

static const uint8_t* SetupPageParams( int page )
{
	return page == 0 ? pageRouting : pageMidi;
}

static void SetParam( _NT_algorithm* self, int paramIndex, int value )
{
	const _NT_parameter& p = self->parameters[paramIndex];
	if ( value < p.min ) value = p.min;
	if ( value > p.max ) value = p.max;
	NT_setParameterFromUi( NT_algorithmIndex( self ), paramIndex + NT_parameterOffset(), value );
}

void	customUi( _NT_algorithm* self, const _NT_uiData& data )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;

	if ( data.controls & kNT_encoderL )
	{
		pThis->currentPage += data.encoders[0];
		if ( pThis->currentPage < 0 ) pThis->currentPage = 0;
		if ( pThis->currentPage >= kNumPages ) pThis->currentPage = kNumPages - 1;
		pThis->setupSelectedItem = 0;
	}

	if ( pThis->currentPage < kNumSetupPages )
	{
		int itemCount = SetupPageItemCount( pThis->currentPage );
		const uint8_t* params = SetupPageParams( pThis->currentPage );

		if ( ( data.controls & kNT_encoderButtonR ) && !( data.lastButtons & kNT_encoderButtonR ) )
		{
			pThis->setupSelectedItem = ( pThis->setupSelectedItem + 1 ) % itemCount;
		}
		if ( data.controls & kNT_encoderR )
		{
			int paramIndex = params[ pThis->setupSelectedItem ];
			int current = self->v[paramIndex];
			SetParam( self, paramIndex, current + data.encoders[1] );
		}
	}
	else
	{
		const uint8_t* params = kBarPageParams[ pThis->currentPage - kNumSetupPages ];
		if ( data.controls & kNT_potL )
		{
			const _NT_parameter& p = self->parameters[ params[0] ];
			SetParam( self, params[0], RoundToInt( p.min + data.pots[0] * ( p.max - p.min ) ) );
		}
		if ( data.controls & kNT_potC )
		{
			const _NT_parameter& p = self->parameters[ params[1] ];
			SetParam( self, params[1], RoundToInt( p.min + data.pots[1] * ( p.max - p.min ) ) );
		}
		if ( data.controls & kNT_potR )
		{
			const _NT_parameter& p = self->parameters[ params[2] ];
			SetParam( self, params[2], RoundToInt( p.min + data.pots[2] * ( p.max - p.min ) ) );
		}
		if ( data.controls & kNT_encoderR )
		{
			int current = self->v[ params[3] ];
			SetParam( self, params[3], current + data.encoders[1] );
		}
	}
}

void	setupUi( _NT_algorithm* self, _NT_float3& pots )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;
	if ( pThis->currentPage < kNumSetupPages )
	{
		pots[0] = pots[1] = pots[2] = 0.0f;
		return;
	}
	const uint8_t* params = kBarPageParams[ pThis->currentPage - kNumSetupPages ];
	for ( int i=0; i<3; ++i )
	{
		const _NT_parameter& p = self->parameters[ params[i] ];
		float range = (float)( p.max - p.min );
		pots[i] = range > 0.0f ? ( self->v[ params[i] ] - p.min ) / range : 0.0f;
	}
}

// ---------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------

static void DrawSetupPage( _drumMachineAlgorithm* pThis, const char* title )
{
	NT_drawText( 4, 12, title, 15, kNT_textLeft, kNT_textNormal );

	int itemCount = SetupPageItemCount( pThis->currentPage );
	const uint8_t* params = SetupPageParams( pThis->currentPage );

	int y = 22;
	for ( int i=0; i<itemCount && y < 62; ++i, y += 8 )
	{
		int paramIndex = params[i];
		bool selected = ( i == pThis->setupSelectedItem );
		NT_drawText( 4, y, pThis->parameters[paramIndex].name, selected ? 15 : 8, kNT_textLeft, kNT_textTiny );

		char buff[16];
		NT_intToString( buff, pThis->v[paramIndex] );
		NT_drawText( 160, y, buff, selected ? 15 : 8, kNT_textLeft, kNT_textTiny );
	}

	NT_drawShapeI( kNT_line, 0, 18, 255, 18, 4 );
}

static void DrawBarPage( _drumMachineAlgorithm* pThis )
{
	int pageIdx = pThis->currentPage - kNumSetupPages;
	const uint8_t* params = kBarPageParams[pageIdx];
	bool bipolar = kBarPageBipolar[pageIdx];

	NT_drawText( 4, 20, kBarPageNames[pageIdx], 15, kNT_textLeft, kNT_textNormal );

	constexpr int y0 = 8;
	constexpr int y1 = 60;
	constexpr int barWidth = 44;
	constexpr int gap = 14;
	constexpr int x0 = 20;

	for ( int slot=0; slot<kNumSlots; ++slot )
	{
		int paramIndex = params[slot];
		const _NT_parameter& p = pThis->parameters[paramIndex];
		int value = pThis->v[paramIndex];

		int bx0 = x0 + slot * ( barWidth + gap );
		int bx1 = bx0 + barWidth;

		NT_drawShapeI( kNT_box, bx0, y0, bx1, y1, 8 );

		float range = (float)( p.max - p.min );
		float t = range > 0.0f ? ( value - p.min ) / range : 0.0f;

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

		NT_drawText( bx0 + barWidth/2, y1 + 9, kSlotNames[slot], 15, kNT_textCentre, kNT_textTiny );

		char buff[16];
		if ( p.unit == kNT_unitEnum && p.enumStrings != NULL )
		{
			int idx = value - p.min;
			if ( idx >= 0 && idx <= ( p.max - p.min ) )
				NT_drawText( bx0 + barWidth/2, ( y0 + y1 ) / 2 + 3, p.enumStrings[idx], 15, kNT_textCentre, kNT_textTiny );
		}
		else
		{
			NT_intToString( buff, value );
			NT_drawText( bx0 + barWidth/2, ( y0 + y1 ) / 2 + 3, buff, 15, kNT_textCentre, kNT_textTiny );
		}
	}
}

bool	draw( _NT_algorithm* self )
{
	_drumMachineAlgorithm* pThis = (_drumMachineAlgorithm*)self;

	if ( pThis->currentPage == 0 )
		DrawSetupPage( pThis, "ROUTING" );
	else if ( pThis->currentPage == 1 )
		DrawSetupPage( pThis, "MIDI" );
	else
		DrawBarPage( pThis );

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
	.midiMessage = midiMessage,
	.tags = kNT_tagInstrument,
	.hasCustomUi = hasCustomUi,
	.customUi = customUi,
	.setupUi = setupUi,
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
