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

#include <vector>

namespace Igorski {
class DownSampler
{
    public:
        DownSampler( int amountOfChannels );
        ~DownSampler();

        void setDownSampleRatio( float value );
        void process( float* sampleBuffer, int bufferSize );

    private:
        float ratio           = 1.f;
        float lastRatio       = 1.f;
        int bufferPos         = 0;
        int sampsInBuffer     = 0;
        float subSampleOffset = 0.0;

        int numChannels;

        void flushBuffers();
        void resetFilters();

};
}

#endif