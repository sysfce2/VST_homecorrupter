/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Igor Zinken - https://www.igorski.nl
 *
 * Adaptation of source provided in the JUCE library:
 * Copyright (c) 2020 - Raw Material Software Limited
 *
 * JUCE is an open source library subject to commercial or open-source
 * licensing.
 *
 * The code included in this file is provided under the terms of the ISC license
 * http://www.isc.org/downloads/software-support-policy/isc-license. Permission
 * To use, copy, modify, and/or distribute this software for any purpose with or
 * without fee is hereby granted provided that the above copyright notice and
 * this permission notice appear in all copies.
 *
 * JUCE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY, AND ALL WARRANTIES, WHETHER
 * EXPRESSED OR IMPLIED, INCLUDING MERCHANTABILITY AND FITNESS FOR PURPOSE, ARE
 * DISCLAIMED.
 */
#include "downsampler.h"
#include "global.h"
#include <limits.h>
#include <algorithm>
#include <numeric>
#include "util.h"

namespace Igorski
{

/* constructor / destructor */

DownSampler::DownSampler()
{
    _in_rate  = ( int ) VST::SAMPLE_RATE;
    _out_rate = _in_rate;
    _num_rate = _in_rate;

    setSampleRate( _in_rate, _in_rate );
}

DownSampler::~DownSampler()
{

}

/* getters / setters */

//static int speex_resampler_set_rate_frac(SpeexResamplerState *st, spx_uint32_t ratio_num, spx_uint32_t ratio_den, spx_uint32_t in_rate, spx_uint32_t out_rate)
void DownSampler::setSampleRate( int inSampleRate, int outSampleRate )
{
    int old_den = _den_rate;
    int fact, i;

    _in_rate = inSampleRate;
    _out_rate = outSampleRate;
    _num_rate = inSampleRate;
    _den_rate = outSampleRate;
    Util::log("here we go", "/Users/igorzinken/Desktop/vst.log");
    Util::log(_num_rate, "/Users/igorzinken/Desktop/vst.log");
    Util::log(_den_rate, "/Users/igorzinken/Desktop/vst.log");
    fact = std::gcd( _num_rate, _den_rate );
    Util::log(fact, "/Users/igorzinken/Desktop/vst.log");

    _num_rate /= fact;
    _den_rate /= fact;
    Util::log(_num_rate, "/Users/igorzinken/Desktop/vst.log");
    Util::log(_den_rate, "/Users/igorzinken/Desktop/vst.log");
    if ( old_den > 0 ) {
        _samp_frac_num = std::min(
            _den_rate - 1,
            mul_div( _samp_frac_num, _den_rate, old_den )
        );
    }
     Util::log("Attempt to update filter", "/Users/igorzinken/Desktop/vst.log");
    updateFilter();
        Util::log("Filter updated.", "/Users/igorzinken/Desktop/vst.log");
}

//static int resampler_basic_interpolate_single(SpeexResamplerState *st, uint32 channel_index, const float *in, uint32 *inputBufferSize, float *out, uint32 *outputBufferSize)

void DownSampler::process( float* inputBuffer, int inputBufferSize, float* outputBuffer, int outputBufferSize )
{
//     struct SpeexResamplerState_ {
//      	uint32 in_rate;
//      	uint32 out_rate;
//      	uint32 num_rate;
//      	uint32 den_rate;
//
//      	int    quality;        // 0 - 10
//      	uint32 nb_channels;    // always 1
//      	uint32 filt_len;
//      	uint32 mem_alloc_size;
//      	uint32 buffer_size;
//      	int          int_advance;
//      	int          frac_advance;
//      	float  cutoff;
//      	uint32 oversample;
//      	int          initialised;
//      	int          started;
//
//      	/* These are per-channel */
//      	uint32  *last_sample;
//      	uint32 *samp_frac_num;
//      	uint32 *magic_samples;
//
//      	float *mem;
//      	float *sinc_table;
//      	uint32 sinc_table_length;
//
//      	int    in_stride;
//      	int    out_stride;
//      };

    _started = 1;

    const int N = _filt_len;
    int out_sample = 0;
    int last_sample = _last_sample;
    int samp_frac_num = _samp_frac_num;
    const int out_stride = _out_stride;
    const int int_advance = _int_advance;
    const int frac_advance = _frac_advance;
    const int den_rate = _den_rate;
    float sum;
                  // Util::log("Process", "/Users/igorzinken/Desktop/vst.log");

    while (!(last_sample >= inputBufferSize || out_sample >= outputBufferSize))
    {
        const float *iptr = &inputBuffer[last_sample];

        const int offset = _samp_frac_num * _oversample / _den_rate;
        const float frac = ((float)(( _samp_frac_num * _oversample ) % _den_rate)) / _den_rate;
        float interp[4];
        int j;
        float accum[4] = { 0, 0, 0, 0 };

        for (j = 0; j<N; j++) {
            const float curr_in = iptr[j];
            accum[0] += MULT16_16(curr_in, _sinc_table[4 + (j + 1) * _oversample - offset - 2]);
            accum[1] += MULT16_16(curr_in, _sinc_table[4 + (j + 1) * _oversample - offset - 1]);
            accum[2] += MULT16_16(curr_in, _sinc_table[4 + (j + 1) * _oversample - offset]);
            accum[3] += MULT16_16(curr_in, _sinc_table[4 + (j + 1) * _oversample - offset + 1]);
        }
        cubic_coef(frac, interp);
        sum = MULT16_32_Q15(interp[0], SHR32(accum[0], 1)) + MULT16_32_Q15(interp[1], SHR32(accum[1], 1)) + MULT16_32_Q15(interp[2], SHR32(accum[2], 1)) + MULT16_32_Q15(interp[3], SHR32(accum[3], 1));
        sum = SATURATE32PSHR(sum, 15, 32767);
    
        outputBuffer[out_stride * out_sample++] = sum;
        last_sample += int_advance;
        samp_frac_num += frac_advance;
        
        if ( samp_frac_num >= den_rate ) {
            samp_frac_num -= den_rate;
            last_sample++;
        }
    }    
    _last_sample = last_sample;
    _samp_frac_num = samp_frac_num;
}

//static int update_filter(SpeexResamplerState *st)
void DownSampler::updateFilter()
{
    int old_length = _filt_len;
    int old_alloc_size = _mem_alloc_size;
    int use_direct;
    int min_sinc_table_length;
    int min_alloc_size;

    _int_advance = _num_rate / _den_rate;
    _frac_advance = _num_rate % _den_rate;
    _oversample = quality_map[_quality].oversample;
    _filt_len = quality_map[_quality].base_length;

    if (_num_rate > _den_rate)
    {
        /* down-sampling */
        _cutoff = quality_map[_quality].downsample_bandwidth * _den_rate / _num_rate;
        _filt_len = mul_div(_filt_len, _num_rate, _den_rate);
        /* Round up to make sure we have a multiple of 8 for SSE */
        _filt_len = ((_filt_len - 1)&(~0x7)) + 8;
        if (2 * _den_rate < _num_rate)
            _oversample >>= 1;
        if (4 * _den_rate < _num_rate)
            _oversample >>= 1;
        if (8 * _den_rate < _num_rate)
            _oversample >>= 1;
        if (16 * _den_rate < _num_rate)
            _oversample >>= 1;
        if (_oversample < 1)
            _oversample = 1;
    }
    else {
        /* up-sampling */
        _cutoff = quality_map[_quality].upsample_bandwidth;
    }
	/* Choose the resampling type that requires the least amount of memory */
//#ifdef RESAMPLE_FULL_SINC_TABLE
//	use_direct = 1;
//	if (INT_MAX / sizeof(float) / _den_rate < _filt_len)
//		return;
//#else
	use_direct = _filt_len*_den_rate <= _filt_len*_oversample + 8
		&& INT_MAX / sizeof(float) / _den_rate >= _filt_len;

//#endif
    if (use_direct)
    {
        min_sinc_table_length = _filt_len * _den_rate;
    }
    else {
        if ((INT_MAX / sizeof(float) - 8) / _oversample < _filt_len)
            return;
        min_sinc_table_length = _filt_len*_oversample + 8;
    }
    if (_sinc_table_length < min_sinc_table_length)
    {
        float* sinc_table = (float *)realloc(_sinc_table, min_sinc_table_length * sizeof(float));
        Util::log("D3", "/Users/igorzinken/Desktop/vst.log");
        if (!sinc_table)
            return;

        _sinc_table = sinc_table;
        _sinc_table_length = min_sinc_table_length;
    }

    if (use_direct)
    {
        uint32 i;
        for (i = 0; i<_den_rate; i++)
        {
            int32 j;
            for (j = 0; j<_filt_len; j++)
            {
                _sinc_table[i*_filt_len + j] = sinc(_cutoff, ((j - (uint32)_filt_len / 2 + 1) - ((float)i) / _den_rate), _filt_len, quality_map[_quality].window_func);
            }
        }
//        if (_quality>8)
//            _resampler_ptr = resampler_basic_direct_double;
//        else
//            _resampler_ptr = resampler_basic_direct_single;
    }
    else {
        uint32 i;
        for (i = -4; i<(uint32)(_oversample*_filt_len + 4); i++)
            _sinc_table[i + 4] = sinc(_cutoff, (i / (float)_oversample - _filt_len / 2), _filt_len, quality_map[_quality].window_func);
//    
//        if (_quality>8)
//            _resampler_ptr = resampler_basic_interpolate_double;
//        else
//            _resampler_ptr = resampler_basic_interpolate_single;
    }
    /* Here's the place where we update the filter memory to take into account
    the change in filter length. It's probably the messiest part of the code
    due to handling of lots of corner cases. */
    
    /* Adding buffer_size to filt_len won't overflow here because filt_len
    could be multiplied by sizeof(float) above. */
    min_alloc_size = _filt_len - 1 + _buffer_size;
    if (min_alloc_size > _mem_alloc_size)
    {
        float *mem;
        if (INT_MAX / sizeof(float) < min_alloc_size)
            return;
        else if (!(mem = (float*)realloc(_mem, min_alloc_size * sizeof(*mem))))
            return;
    
        _mem = mem;
        _mem_alloc_size = min_alloc_size;
    }
    if (!_started)
    {
        uint32 i;
        for (i = 0; i<_mem_alloc_size; i++)
            _mem[i] = 0;
        /*speex_warning("reinit filter");*/
    }
    else if (_filt_len > old_length)
    {
        uint32 i;
        /* Increase the filter length */
        /*speex_warning("increase filter size");*/
        //for (i = _nb_channels; i--;)
        //{
            uint32 j;
            uint32 olen = old_length;
            /*if (_magic_samples)*/
            {
                /* Try and remove the magic samples as if nothing had happened */
    
                /* FIXME: This is wrong but for now we need it to avoid going over the array bounds */
                olen = old_length + 2 * _magic_samples;
                for (j = old_length - 1 + _magic_samples; j--;)
                    _mem[_mem_alloc_size + j + _magic_samples] = _mem[old_alloc_size + j];
                for (j = 0; j<_magic_samples; j++)
                    _mem[_mem_alloc_size + j] = 0;
                _magic_samples = 0;
            }
            if (_filt_len > olen)
            {
                /* If the new filter length is still bigger than the "augmented" length */
                /* Copy data going backward */
                for (j = 0; j<olen - 1; j++)
                    _mem[_mem_alloc_size + (_filt_len - 2 - j)] = _mem[_mem_alloc_size + (olen - 2 - j)];
                /* Then put zeros for lack of anything better */
                for (; j<_filt_len - 1; j++)
                    _mem[_mem_alloc_size + (_filt_len - 2 - j)] = 0;
                /* Adjust last_sample */
                _last_sample += (_filt_len - olen) / 2;
            }
            else {
                /* Put back some of the magic! */
                _magic_samples = (olen - _filt_len) / 2;
                for (j = 0; j<_filt_len - 1 + _magic_samples; j++)
                    _mem[_mem_alloc_size + j] = _mem[_mem_alloc_size + j + _magic_samples];
            }
        //}
    }
    else if (_filt_len < old_length)
    {
        uint32 i;
        /* Reduce filter length, this a bit tricky. We need to store some of the memory as "magic"
        samples so they can be used directly as input the next time(s) */
        //for (i = 0; i<_nb_channels; i++)
        //{
            uint32 j;
            uint32 old_magic = _magic_samples;
            _magic_samples = (old_length - _filt_len) / 2;
            /* We must copy some of the memory that's no longer used */
            /* Copy data going backward */
            for (j = 0; j<_filt_len - 1 + _magic_samples + old_magic; j++)
                _mem[_mem_alloc_size + j] = _mem[_mem_alloc_size + j + _magic_samples];
            _magic_samples += old_magic;
        //}
    }
}

void DownSampler::cubic_coef(float frac, float interp[4])
{
    /* Compute interpolation coefficients. I'm not sure whether this corresponds to cubic interpolation
    but I know it's MMSE-optimal on a sinc */
    interp[0] = -0.16667f*frac + 0.16667f*frac*frac*frac;
    interp[1] = frac + 0.5f*frac*frac - 0.5f*frac*frac*frac;
    /*interp[2] = 1.f - 0.5f*frac - frac*frac + 0.5f*frac*frac*frac;*/
    interp[3] = -0.33333f*frac + 0.5f*frac*frac - 0.16667f*frac*frac*frac;
    /* Just to make sure we don't have rounding problems */
    interp[2] = 1. - interp[0] - interp[1] - interp[3];
}

}