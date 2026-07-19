// Tuner + scope: analyses an audio input for pitch (note + frequency) and
// displays it alongside a live oscilloscope trace of the waveform.

#include <math.h>
#include <string.h>
#include <algorithm>
#include <new>
#include <distingnt/api.h>

// Frequencies of MIDI notes 0-127 (A4 = note 69 = 440Hz, equal temperament).
// Used as a lookup table so pitch-to-note conversion needs no runtime log()
// call - the disting NT firmware only resolves a limited set of libm
// symbols against loaded plugins, and which exact symbol a given call
// compiles down to (log vs logf etc.) isn't reliably predictable from the
// source spelling alone, so avoiding the dependency entirely is safest.
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

// Finds the nearest MIDI note to freq by binary search over the
// monotonically increasing table above, then picking whichever neighbour is
// geometrically closer (comparing freq*freq against the product of the two
// candidate frequencies is equivalent to comparing freq against their
// geometric mean, without needing a sqrt call).
static int nearestMidiNote( float freq )
{
	int lo = 0, hi = 127;
	while ( lo < hi )
	{
		int mid = ( lo + hi ) / 2;
		if ( kMidiNoteFreq[mid] < freq )
			lo = mid + 1;
		else
			hi = mid;
	}
	if ( lo > 0 )
	{
		float below = kMidiNoteFreq[lo-1];
		float above = kMidiNoteFreq[lo];
		if ( freq * freq < below * above )
			return lo - 1;
	}
	return lo;
}

enum
{
	kParamInput,

	kNumParams,
};

static const _NT_parameter parameters[] = {
	NT_PARAMETER_AUDIO_INPUT( "Input", 1, 1 )
};

static const uint8_t page1[] = { kParamInput };

static const _NT_parameterPage pages[] = {
	{ .name = "Routing", .numParams = ARRAY_SIZE(page1), .params = page1 },
};

static const _NT_parameterPages parameterPages = {
	.numPages = ARRAY_SIZE(pages),
	.pages = pages,
};

struct _tunerAlgorithm : public _NT_algorithm
{
	_tunerAlgorithm() {}
	~_tunerAlgorithm() {}

	// Raw samples for the scope display - a circular buffer of the most
	// recent input, read back oldest-to-newest via scopeSample().
	static constexpr int kScopeBufferSize = 1024;

	// Pitch is estimated from a decimated, continuously-written circular
	// copy of the input (read back via pitchSample(), same convention as
	// scopeSample()). Decimating keeps each single-lag comparison cheap.
	static constexpr int kDecimation      = 4;
	static constexpr int kPitchBufferSize = 1024;
	static constexpr int kYinWindow       = 512;	// samples compared at each lag
	static constexpr int kYinMinTau       = 8;		// ~ decimatedRate/1500Hz
	static constexpr int kYinMaxTau       = 400;	// ~ decimatedRate/30Hz
	static_assert( kYinWindow + kYinMaxTau <= kPitchBufferSize );

	float*	scopeBuffer = NULL;	// [kScopeBufferSize], points into dram
	float*	pitchBuffer = NULL;	// [kPitchBufferSize], points into dram

	int		scopeWrite = 0;
	int		pitchWrite = 0;

	// Incremental YIN state: step() evaluates exactly one lag per call
	// (see the comment in step()) instead of running the whole search in
	// one burst, so a single audio callback never has to absorb the full
	// ~55000-operation cost of a complete pitch analysis in one go.
	int		pitchTau = kYinMinTau;
	float	pitchBestVal = 0.0f;
	int		pitchBestTau = kYinMinTau;

	bool	hasPitch = false;
	float	frequency = 0.0f;
	int		noteIndex = 0;		// 0=C .. 11=B
	int		octave = 4;

	float scopeSample( int i ) const
	{
		return scopeBuffer[ ( scopeWrite + i ) % kScopeBufferSize ];
	}

	// kPitchBufferSize is a power of two so this compiles to a cheap mask,
	// not an actual division.
	float pitchSample( int i ) const
	{
		return pitchBuffer[ ( pitchWrite + i ) % kPitchBufferSize ];
	}
};

void	calculateRequirements( _NT_algorithmRequirements& req, const int32_t* specifications )
{
	req.numParameters = ARRAY_SIZE(parameters);
	req.sram = sizeof(_tunerAlgorithm);
	req.dram = ( _tunerAlgorithm::kScopeBufferSize + _tunerAlgorithm::kPitchBufferSize ) * sizeof(float);
	req.dtc = 0;
	req.itc = 0;
}

_NT_algorithm*	construct( const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req, const int32_t* specifications )
{
	_tunerAlgorithm* alg = new (ptrs.sram) _tunerAlgorithm();

	float* dram = (float*)ptrs.dram;
	alg->scopeBuffer = dram;
	alg->pitchBuffer = dram + _tunerAlgorithm::kScopeBufferSize;
	memset( dram, 0, req.dram );

	alg->parameters = parameters;
	alg->parameterPages = &parameterPages;
	return alg;
}

// Sum of squared differences between the window starting at the oldest
// buffered sample and the window starting `tau` samples later, i.e. the
// core YIN difference function, evaluated for a single lag.
static float pitchDifferenceAt( const _tunerAlgorithm* pThis, int tau )
{
	float sum = 0.0f;
	for ( int j=0; j<_tunerAlgorithm::kYinWindow; ++j )
	{
		float d = pThis->pitchSample( j ) - pThis->pitchSample( j + tau );
		sum += d * d;
	}
	return sum;
}

// Called once a full lag sweep (kYinMinTau..kYinMaxTau) has completed.
// pThis->pitchBestTau/pitchBestVal hold the winning lag found incrementally
// in step(); this does the remaining (cheap, one-off) work: a silence gate,
// parabolic interpolation for sub-sample precision, and the note lookup.
static void finalizeAnalysis( _tunerAlgorithm* pThis )
{
	const int W = _tunerAlgorithm::kYinWindow;
	const int minTau = _tunerAlgorithm::kYinMinTau;
	const int maxTau = _tunerAlgorithm::kYinMaxTau;

	// Compared as mean-square vs. a squared threshold so this doesn't need
	// a sqrt call - the firmware's plugin loader only resolves a limited
	// set of libm symbols (see the kMidiNoteFreq comment above).
	float meanSquare = 0.0f;
	for ( int i=0; i<W; ++i )
	{
		float s = pThis->pitchSample( i );
		meanSquare += s * s;
	}
	meanSquare /= W;
	constexpr float kSilenceThresholdSquared = 0.005f * 0.005f;
	if ( meanSquare < kSilenceThresholdSquared )
	{
		pThis->hasPitch = false;
		return;
	}

	int bestTau = pThis->pitchBestTau;

	// Parabolic interpolation around the winning lag for sub-sample precision.
	float tauRefined = (float)bestTau;
	if ( bestTau > minTau && bestTau < maxTau )
	{
		float dm1 = pitchDifferenceAt( pThis, bestTau - 1 );
		float d0  = pThis->pitchBestVal;
		float dp1 = pitchDifferenceAt( pThis, bestTau + 1 );
		float denom = dm1 - 2.0f * d0 + dp1;
		if ( denom * denom > 1.0e-18f )
			tauRefined = bestTau + 0.5f * ( dm1 - dp1 ) / denom;
	}

	float decimatedRate = NT_globals.sampleRate / (float)_tunerAlgorithm::kDecimation;
	float freq = decimatedRate / tauRefined;

	pThis->frequency = freq;
	pThis->hasPitch = true;

	int nearest = nearestMidiNote( freq );
	pThis->noteIndex = ( ( nearest % 12 ) + 12 ) % 12;
	pThis->octave = nearest / 12 - 1;
}

void 	step( _NT_algorithm* self, float* busFrames, int numFramesBy4 )
{
	_tunerAlgorithm* pThis = (_tunerAlgorithm*)self;
	int numFrames = numFramesBy4 * 4;
	const float* in = busFrames + ( pThis->v[kParamInput] - 1 ) * numFrames;

	for ( int i=0; i<numFrames; ++i )
	{
		pThis->scopeBuffer[ pThis->scopeWrite ] = in[i];
		pThis->scopeWrite = ( pThis->scopeWrite + 1 ) % _tunerAlgorithm::kScopeBufferSize;
	}

	// numFrames is always a multiple of 4 (it's numFramesBy4 * 4), so this
	// decimation never needs to carry a partial sum across step() calls.
	for ( int i=0; i<numFrames; i+=4 )
	{
		float avg = 0.25f * ( in[i] + in[i+1] + in[i+2] + in[i+3] );
		pThis->pitchBuffer[ pThis->pitchWrite ] = avg;
		pThis->pitchWrite = ( pThis->pitchWrite + 1 ) % _tunerAlgorithm::kPitchBufferSize;
	}

	// Evaluate exactly one YIN lag per step() call. A full sweep costs
	// ~55000 operations; doing it all at once caused audible clicks
	// despite negligible *average* CPU load, because a multi-millisecond
	// spike inside a single audio callback can blow that block's real-time
	// deadline even though the average across many blocks looks tiny.
	// Spreading it to one cheap, constant-cost lag per call fixes that.
	float d = pitchDifferenceAt( pThis, pThis->pitchTau );
	if ( pThis->pitchTau == _tunerAlgorithm::kYinMinTau || d < pThis->pitchBestVal )
	{
		pThis->pitchBestVal = d;
		pThis->pitchBestTau = pThis->pitchTau;
	}
	pThis->pitchTau++;
	if ( pThis->pitchTau > _tunerAlgorithm::kYinMaxTau )
	{
		finalizeAnalysis( pThis );
		pThis->pitchTau = _tunerAlgorithm::kYinMinTau;
	}
}

static void drawScope( _tunerAlgorithm* pThis )
{
	constexpr int x0 = 132;
	constexpr int x1 = 254;
	constexpr int y0 = 6;
	constexpr int y1 = 58;
	constexpr int width  = x1 - x0;
	constexpr int midY   = ( y0 + y1 ) / 2;
	constexpr int N      = _tunerAlgorithm::kScopeBufferSize;

	NT_drawShapeI( kNT_box, x0 - 2, y0 - 2, x1 + 2, y1 + 2, 3 );

	// Trigger on the first upward zero-crossing so a periodic waveform
	// holds still instead of visibly drifting frame to frame.
	int start = 0;
	for ( int i=1; i<N/2; ++i )
	{
		if ( pThis->scopeSample( i-1 ) <= 0.0f && pThis->scopeSample( i ) > 0.0f )
		{
			start = i;
			break;
		}
	}

	int displaySamples = std::min( N - start, N/2 );

	float peak = 0.0f;
	for ( int i=0; i<displaySamples; ++i )
	{
		float v = pThis->scopeSample( start + i );
		peak = std::max( peak, v < 0.0f ? -v : v );
	}
	peak = std::max( peak, 0.01f );
	float scale = ( ( y1 - y0 ) * 0.5f - 1.0f ) / peak;

	float prevX = 0.0f, prevY = 0.0f;
	for ( int px=0; px<width; ++px )
	{
		int i = px * displaySamples / width;
		float v = pThis->scopeSample( start + i );
		float x = x0 + px;
		float y = midY - v * scale;
		if ( px > 0 )
			NT_drawShapeF( kNT_line, prevX, prevY, x, y, 15 );
		prevX = x;
		prevY = y;
	}
}

bool	draw( _NT_algorithm* self )
{
	_tunerAlgorithm* pThis = (_tunerAlgorithm*)self;

	if ( pThis->hasPitch )
	{
		static const char* names[12] = { "C","C#","D","D#","E","F","F#","G","G#","A","A#","B" };

		char noteStr[8];
		strcpy( noteStr, names[ pThis->noteIndex ] );
		char octBuff[4];
		NT_intToString( octBuff, pThis->octave );
		strcat( noteStr, octBuff );
		NT_drawText( 8, 44, noteStr, 15, kNT_textLeft, kNT_textLarge );

		char freqStr[16];
		int len = NT_floatToString( freqStr, pThis->frequency, 1 );
		strcpy( freqStr + len, " Hz" );
		NT_drawText( 8, 58, freqStr, 12, kNT_textLeft, kNT_textNormal );
	}
	else
	{
		NT_drawText( 8, 44, "--", 8, kNT_textLeft, kNT_textLarge );
	}

	drawScope( pThis );

	return true;	// suppress the standard parameter line - use the full screen
}

static const _NT_factory factory =
{
	.guid = NT_MULTICHAR( 'T', 'u', 'n', 'r' ),
	.name = "Tuner + Scope",
	.description = "Note/frequency tuner with an oscilloscope display",
	.numSpecifications = 0,
	.calculateRequirements = calculateRequirements,
	.construct = construct,
	.parameterChanged = NULL,
	.step = step,
	.draw = draw,
	.tags = kNT_tagUtility,
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
