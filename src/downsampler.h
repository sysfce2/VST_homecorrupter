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
#ifndef __DOWNSAMPLER_H_INCLUDED__
#define __DOWNSAMPLER_H_INCLUDED__

#include "global.h"

namespace Igorski {

static const double kaiser12_table[68] = {
    0.99859849, 1.00000000, 0.99859849, 0.99440475, 0.98745105, 0.97779076,
    0.96549770, 0.95066529, 0.93340547, 0.91384741, 0.89213598, 0.86843014,
    0.84290116, 0.81573067, 0.78710866, 0.75723148, 0.72629970, 0.69451601,
    0.66208321, 0.62920216, 0.59606986, 0.56287762, 0.52980938, 0.49704014,
    0.46473455, 0.43304576, 0.40211431, 0.37206735, 0.34301800, 0.31506490,
    0.28829195, 0.26276832, 0.23854851, 0.21567274, 0.19416736, 0.17404546,
    0.15530766, 0.13794294, 0.12192957, 0.10723616, 0.09382272, 0.08164178,
    0.07063950, 0.06075685, 0.05193064, 0.04409466, 0.03718069, 0.03111947,
    0.02584161, 0.02127838, 0.01736250, 0.01402878, 0.01121463, 0.00886058,
    0.00691064, 0.00531256, 0.00401805, 0.00298291, 0.00216702, 0.00153438,
    0.00105297, 0.00069463, 0.00043489, 0.00025272, 0.00013031, 0.0000527734,
    0.00001000, 0.00000000 };
/*
static const double kaiser12_table[36] = {
0.99440475, 1.00000000, 0.99440475, 0.97779076, 0.95066529, 0.91384741,
0.86843014, 0.81573067, 0.75723148, 0.69451601, 0.62920216, 0.56287762,
0.49704014, 0.43304576, 0.37206735, 0.31506490, 0.26276832, 0.21567274,
0.17404546, 0.13794294, 0.10723616, 0.08164178, 0.06075685, 0.04409466,
0.03111947, 0.02127838, 0.01402878, 0.00886058, 0.00531256, 0.00298291,
0.00153438, 0.00069463, 0.00025272, 0.0000527734, 0.00000500, 0.00000000};
*/
static const double kaiser10_table[36] = {
    0.99537781, 1.00000000, 0.99537781, 0.98162644, 0.95908712, 0.92831446,
    0.89005583, 0.84522401, 0.79486424, 0.74011713, 0.68217934, 0.62226347,
    0.56155915, 0.50119680, 0.44221549, 0.38553619, 0.33194107, 0.28205962,
    0.23636152, 0.19515633, 0.15859932, 0.12670280, 0.09935205, 0.07632451,
    0.05731132, 0.04193980, 0.02979584, 0.02044510, 0.01345224, 0.00839739,
    0.00488951, 0.00257636, 0.00115101, 0.00035515, 0.00000000, 0.00000000 };

static const double kaiser8_table[36] = {
    0.99635258, 1.00000000, 0.99635258, 0.98548012, 0.96759014, 0.94302200,
    0.91223751, 0.87580811, 0.83439927, 0.78875245, 0.73966538, 0.68797126,
    0.63451750, 0.58014482, 0.52566725, 0.47185369, 0.41941150, 0.36897272,
    0.32108304, 0.27619388, 0.23465776, 0.19672670, 0.16255380, 0.13219758,
    0.10562887, 0.08273982, 0.06335451, 0.04724088, 0.03412321, 0.02369490,
    0.01563093, 0.00959968, 0.00527363, 0.00233883, 0.00050000, 0.00000000 };

static const double kaiser6_table[36] = {
    0.99733006, 1.00000000, 0.99733006, 0.98935595, 0.97618418, 0.95799003,
    0.93501423, 0.90755855, 0.87598009, 0.84068475, 0.80211977, 0.76076565,
    0.71712752, 0.67172623, 0.62508937, 0.57774224, 0.53019925, 0.48295561,
    0.43647969, 0.39120616, 0.34752997, 0.30580127, 0.26632152, 0.22934058,
    0.19505503, 0.16360756, 0.13508755, 0.10953262, 0.08693120, 0.06722600,
    0.05031820, 0.03607231, 0.02432151, 0.01487334, 0.00752000, 0.00000000 };

struct FuncDef {
    const double *table;
    int oversample;
};

static const struct FuncDef _KAISER12 = { kaiser12_table, 64 };
#define KAISER12 (&_KAISER12)
/*static struct FuncDef _KAISER12 = {kaiser12_table, 32};
#define KAISER12 (&_KAISER12)*/
static const struct FuncDef _KAISER10 = { kaiser10_table, 32 };
#define KAISER10 (&_KAISER10)
static const struct FuncDef _KAISER8 = { kaiser8_table, 32 };
#define KAISER8 (&_KAISER8)
static const struct FuncDef _KAISER6 = { kaiser6_table, 32 };
#define KAISER6 (&_KAISER6)

#define MULT16_16(a,b)     ((float)(a)*(float)(b))
#define MULT16_32_Q15(a,b)     ((a)*(b))
#define SHR32(a,shift) (a)
#define SATURATE32PSHR(x,shift,a) (x)

class DownSampler
{
    struct QualityMapping {
        int base_length;
        int oversample;
        float downsample_bandwidth;
        float upsample_bandwidth;
        const struct FuncDef *window_func;
    };

    /* This table maps conversion quality to internal parameters. There are two
    reasons that explain why the up-sampling bandwidth is larger than the
    down-sampling bandwidth:
    1) When up-sampling, we can assume that the spectrum is already attenuated
    close to the Nyquist rate (from an A/D or a previous resampling filter)
    2) Any aliasing that occurs very close to the Nyquist rate will be masked
    by the sinusoids/noise just below the Nyquist rate (guaranteed only for
    up-sampling).
    */
    static constexpr struct QualityMapping quality_map[11] = {
        { 8, 4, 0.830f, 0.860f, KAISER6 }, /* Q0 */
        { 16, 4, 0.850f, 0.880f, KAISER6 }, /* Q1 */
        { 32, 4, 0.882f, 0.910f, KAISER6 }, /* Q2 */  /* 82.3% cutoff ( ~60 dB stop) 6  */
        { 48, 8, 0.895f, 0.917f, KAISER8 }, /* Q3 */  /* 84.9% cutoff ( ~80 dB stop) 8  */
        { 64, 8, 0.921f, 0.940f, KAISER8 }, /* Q4 */  /* 88.7% cutoff ( ~80 dB stop) 8  */
        { 80, 16, 0.922f, 0.940f, KAISER10 }, /* Q5 */  /* 89.1% cutoff (~100 dB stop) 10 */
        { 96, 16, 0.940f, 0.945f, KAISER10 }, /* Q6 */  /* 91.5% cutoff (~100 dB stop) 10 */
        { 128, 16, 0.950f, 0.950f, KAISER10 }, /* Q7 */  /* 93.1% cutoff (~100 dB stop) 10 */
        { 160, 16, 0.960f, 0.960f, KAISER10 }, /* Q8 */  /* 94.5% cutoff (~100 dB stop) 10 */
        { 192, 32, 0.968f, 0.968f, KAISER12 }, /* Q9 */  /* 95.5% cutoff (~100 dB stop) 10 */
        { 256, 32, 0.975f, 0.975f, KAISER12 }, /* Q10 */ /* 96.6% cutoff (~100 dB stop) 10 */
    };

    public:
        DownSampler();
        ~DownSampler();

        void setSampleRate( int inSampleRate, int outSampleRate );
        void process( float* inputBuffer, int inputBufferSize, float* outputBuffer, int outputBufferSize );

    private:
        int _in_rate;  // is sample rate of source
        int _out_rate;  // is sample rate of destination
        int _num_rate = 0;  // is ratio numerator
        int _den_rate = 0;  // is ratio denomenator
        int _quality = 8;        // 0 - 10
        //      	uint32 nb_channels;    // always 1
        int _filt_len = 0;
        int _mem_alloc_size = 0;
        int _buffer_size = 160;
        int    _int_advance = 0;
        int    _frac_advance = 0;
        float  _cutoff;
        int _oversample;
//        int    _initialised;
        int    _started = 0;

        /* These are per-channel */
        int _last_sample = 0;
        int _samp_frac_num = 0;
        int _magic_samples = 0;

        float* _mem = nullptr;
        float* _sinc_table = nullptr;
        int _sinc_table_length = 0;

        int _in_stride = 1;
        int _out_stride = 1;


        void updateFilter();
        void cubic_coef( float frac, float interp[ 4 ]);

        inline float sinc(float cutoff, float x, int N, const struct FuncDef *window_func)
        {
            /*fprintf (stderr, "%f ", x);*/
            float xx = x * cutoff;
            if (fabs(x)<1e-6)
                return cutoff;
            else if (fabs(x) > .5*N)
                return 0;
            /*FIXME: Can it really be any slower than this? */
            return cutoff*sin(VST::PI*xx) / (VST::PI*xx) * compute_func(fabs(2.*x / N), window_func);
        }

        inline double compute_func(float x, const struct FuncDef *func)
        {
        	float y, frac;
        	double interp[4];
        	int ind;
        	y = x*func->oversample;
        	ind = (int)floor(y);
        	frac = (y - ind);
        	/* CSE with handle the repeated powers */
        	interp[3] = -0.1666666667*frac + 0.1666666667*(frac*frac*frac);
        	interp[2] = frac + 0.5*(frac*frac) - 0.5*(frac*frac*frac);
        	/*interp[2] = 1.f - 0.5f*frac - frac*frac + 0.5f*frac*frac*frac;*/
        	interp[0] = -0.3333333333*frac + 0.5*(frac*frac) - 0.1666666667*(frac*frac*frac);
        	/* Just to make sure we don't have rounding problems */
        	interp[1] = 1.f - interp[3] - interp[2] - interp[0];

        	/*sum = frac*accum[1] + (1-frac)*accum[2];*/
        	return interp[0] * func->table[ind] + interp[1] * func->table[ind + 1] + interp[2] * func->table[ind + 2] + interp[3] * func->table[ind + 3];
        }

        inline int mul_div(int number, int numerator, int denominator) {
            long long ret = number;
            ret *= numerator;
            ret /= denominator;
            return (int) ret;
        }
};
}

#endif