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
#include <cmath>
#include <algorithm>

namespace Igorski
{

/* constructor / destructor */

DownSampler::DownSampler()
{
    _lowpassFilter = new LowPassFilter();
}

DownSampler::~DownSampler()
{
    delete _lowpassFilter;
}

/* getters / setters */

//void ResamplingAudioSource::setResamplingRatio (const double samplesInPerOutputSample)
void DownSampler::setRatio( float value )
{
    ratio = std::max( 0.f, value );
    flushBuffers();
}

/*
void ResamplingAudioSource::prepareToPlay (int samplesPerBlockExpected, double sampleRate)
{
    const SpinLock::ScopedLockType sl (ratioLock);

    auto scaledBlockSize = roundToInt (samplesPerBlockExpected * ratio);
    input->prepareToPlay (scaledBlockSize, sampleRate * ratio);

    buffer.setSize (numChannels, scaledBlockSize + 32);

    filterStates.calloc (numChannels);
    srcBuffers.calloc (numChannels);
    destBuffers.calloc (numChannels);
    createLowPass (ratio);

    flushBuffers();
}
*/

/* public methods */

void DownSampler::flushBuffers()
{
    bufferPos = 0;
    sampsInBuffer = 0;
    subSampleOffset = 0.0;
    _lowpassFilter->resetFilter();
}

/*
void ResamplingAudioSource::releaseResources()
{
    input->releaseResources();
    buffer.setSize (numChannels, 0);
}
*/

//void ResamplingAudioSource::getNextAudioBlock (const AudioSourceChannelInfo& info)
void DownSampler::process( float* inputBuffer, int bufferSize, float* outputBuffer, int outputBufferSize )
{
    float localRatio = ratio;
    if ( lastRatio != localRatio ) {
        _lowpassFilter->setRatio( localRatio );
        lastRatio = localRatio;
    }
    const int sampsNeeded = ( int )( bufferSize * localRatio ) + 3;

//    if ( bufferSize < sampsNeeded + 8 ) {
//        bufferPos %= bufferSize;
//        bufferSize = sampsNeeded + 32;
//        buffer.setSize( buffer.getNumChannels(), bufferSize, true, true );
//    }

    bufferPos %= bufferSize;

    int endOfBufferPos = bufferPos + sampsInBuffer;

    while ( sampsNeeded > sampsInBuffer )
    {
        endOfBufferPos %= bufferSize;

        int numToDo = std::min( sampsNeeded - sampsInBuffer, bufferSize - endOfBufferPos );

        if ( localRatio > 1.0001 ) {
            // for down-sampling, pre-apply the filter..
            _lowpassFilter->applyFilter( inputBuffer, bufferSize );
        }
        sampsInBuffer  += numToDo;
        endOfBufferPos += numToDo;
    }

//    for ( int channel = 0; channel < channelsToProcess; ++channel ) {
//        destBuffers[ channel ] = info.buffer->getWritePointer( channel, info.startSample );
//        srcBuffers[ channel ]  = buffer.getReadPointer( channel );
//    }
//              for (int i = 0; i <bufferSize;++i)
//              outputBuffer[i]=inputBuffer[i];
//          return; // QQQ
    int nextPos = ( bufferPos + 1 ) % bufferSize;

    for ( int i = 0, m = outputBufferSize; --m >= 0; ++i ) {
        if ( sampsInBuffer <= 0 || nextPos == endOfBufferPos ) {
            break;
        }

        const float alpha = ( float ) subSampleOffset;

        const float curSample  = inputBuffer[ bufferPos ];
        const float nextSample = inputBuffer[ nextPos ];

        outputBuffer[ i ] = curSample + subSampleOffset * ( nextSample - curSample );

        subSampleOffset += localRatio;

        while ( subSampleOffset >= 1.f )
        {
            if ( ++bufferPos >= bufferSize ) {
                bufferPos = 0;
            }
            --sampsInBuffer;

            nextPos = ( bufferPos + 1 ) % bufferSize;
            subSampleOffset -= 1.f;
        }
    }

//    if (localRatio <= 1.0001 && info.numSamples > 0)
//    {
//        // if the filter's not currently being applied, keep it stoked with the last couple of samples to avoid discontinuities
//        const float* const endOfBuffer = info.buffer->getReadPointer (i, info.startSample + info.numSamples - 1);
//        FilterState& fs = filterStates[i];
//
//        if (info.numSamples > 1)
//        {
//            fs.y2 = fs.x2 = *(endOfBuffer - 1);
//        }
//        else
//        {
//            fs.y2 = fs.y1;
//            fs.x2 = fs.x1;
//        }
//
//        fs.y1 = fs.x1 = *endOfBuffer;
//    }
}

}