// Starter template for a new disting NT algorithm.
// Copy this file, rename it, and fill in the TODOs.

#include <math.h>
#include <new>
#include <distingnt/api.h>

// TODO: rename this struct and add whatever per-instance state you need.
struct _templateAlgorithm : public _NT_algorithm
{
	_templateAlgorithm() {}
	~_templateAlgorithm() {}
};

enum
{
	kParamInput,
	kParamOutput,
	kParamOutputMode,
	// TODO: add your own parameter indices here.
};

static const _NT_parameter parameters[] = {
	NT_PARAMETER_AUDIO_INPUT( "Input", 1, 1 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Output", 1, 2 )
	// TODO: add your own parameters here.
};

static const uint8_t page1[] = { kParamInput, kParamOutput, kParamOutputMode };

static const _NT_parameterPage pages[] = {
	{ .name = "Routing", .numParams = ARRAY_SIZE(page1), .params = page1 },
};

static const _NT_parameterPages parameterPages = {
	.numPages = ARRAY_SIZE(pages),
	.pages = pages,
};

void	calculateRequirements( _NT_algorithmRequirements& req, const int32_t* specifications )
{
	req.numParameters = ARRAY_SIZE(parameters);
	req.sram = sizeof(_templateAlgorithm);
	req.dram = 0;
	req.dtc = 0;
	req.itc = 0;
}

_NT_algorithm*	construct( const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req, const int32_t* specifications )
{
	_templateAlgorithm* alg = new (ptrs.sram) _templateAlgorithm();
	alg->parameters = parameters;
	alg->parameterPages = &parameterPages;
	return alg;
}

void 	step( _NT_algorithm* self, float* busFrames, int numFramesBy4 )
{
	_templateAlgorithm* pThis = (_templateAlgorithm*)self;
	int numFrames = numFramesBy4 * 4;
	const float* in = busFrames + ( pThis->v[kParamInput] - 1 ) * numFrames;
	float* out = busFrames + ( pThis->v[kParamOutput] - 1 ) * numFrames;
	bool replace = pThis->v[kParamOutputMode];

	// TODO: replace this passthrough with your DSP.
	for ( int i=0; i<numFrames; ++i )
	{
		if ( replace )
			out[i] = in[i];
		else
			out[i] += in[i];
	}
}

static const _NT_factory factory =
{
	// TODO: pick a GUID unique among your own plugins ('T','p','l','1' is just a placeholder).
	.guid = NT_MULTICHAR( 'T', 'p', 'l', '1' ),
	.name = "Template",
	.description = "Starter template - copy this file to begin a new algorithm",
	.numSpecifications = 0,
	.calculateRequirements = calculateRequirements,
	.construct = construct,
	.parameterChanged = NULL,
	.step = step,
	.draw = NULL,
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
