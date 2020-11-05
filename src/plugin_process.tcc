/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Igor Zinken - https://www.igorski.nl
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <algorithm>

namespace Igorski
{
template <typename SampleType>
void PluginProcess::process( SampleType** inBuffer, SampleType** outBuffer, int numInChannels, int numOutChannels,
                             int bufferSize, uint32 sampleFramesSize ) {

    float fBufferSize = ( float ) bufferSize;
    int maxBufferPos  = bufferSize - 1;

    // input and output buffers can be float or double as defined
    // by the templates SampleType value. Internally we process
    // audio as floats

    SampleType inSample;
    int32 i;

    bool mixDry = _dryMix != 0.f;
    bool mustDownSample = _downSampleAmount > 1.f;

    SampleType dryMix = ( SampleType ) _dryMix;
    SampleType wetMix = ( SampleType ) _wetMix;

    prepareMixBuffers( inBuffer, numInChannels, bufferSize );

    float readPointer;
    int writePointer;
    int recordMax = _maxRecordBufferSize - 1;

    int t, t2;
    float incr, frac, s1, s2;

    // cache oscillator positions (are reset for each channel)

    float downSampleLfoAcc   = _downSampleLfo->getAccumulator();
    float playbackRateLfoAcc = _playbackRateLfo->getAccumulator();

    float subSampleOffset;
    int bufferPos;

    // dithering variables
    int r1 = 0;
    int r2 = 0;

    for ( int32 c = 0; c < numInChannels; ++c )
    {
        readPointer  = _readPointer;
        writePointer = _writePointer;

        subSampleOffset = _subSampleOffset;

        SampleType* channelInBuffer  = inBuffer[ c ];
        SampleType* channelOutBuffer = outBuffer[ c ];
        float* channelRecordBuffer   = _recordBuffer->getBufferForChannel( c );
        float* channelPreMixBuffer   = _preMixBuffer->getBufferForChannel( c );
        float* channelPostMixBuffer  = _postMixBuffer->getBufferForChannel( c );

        _downSampleLfo->setAccumulator( downSampleLfoAcc );
        _playbackRateLfo->setAccumulator( playbackRateLfoAcc );

        float lastSample = _lastSamples[ c ];

        // write input into the record buffer (converting to float when necessary)

        for ( i = 0; i < bufferSize; ++i, ++writePointer ) {
            if ( writePointer > recordMax ) {
                writePointer = 0;
            }
            channelRecordBuffer[ writePointer ] = ( float ) channelInBuffer[ i ];
        }

        // write current read range into the premix buffer, downsampling as necessary

        for ( i = 0; i < bufferSize; ++i ) {
            t  = ( int ) readPointer;
            t2 = std::min( recordMax, t + 1 );

            frac = readPointer - t;

            s1 = channelRecordBuffer[ t ];
            s2 = channelRecordBuffer[ t2 ];

            float outSample = ( s1 + ( s2 - s1 ) * frac );

            channelPreMixBuffer[ i ] = outSample;

            // run the oscillator, note we multiply by .5 and add .5 to make the LFO's bipolar waveforms unipolar

            if ( _hasPlaybackRateLfo ) {
                float lfoValue = _playbackRateLfo->peek() * .5f + .5f;
                _tempPlaybackRate = std::min( _playbackRateLfoMax, _playbackRateLfoMin + _playbackRateLfoRange * lfoValue );
            }

            if (( readPointer += _tempPlaybackRate ) > recordMax ) {
                readPointer = 0.f;
            }
        }

        // apply down sampling
        bufferPos = _bufferPos;
        int nextPos = ( bufferPos + 1 ) % bufferSize;

        if ( mustDownSample ) {


            /* ------ DECIMATOR ----- */
            /*
            for ( i = 0; i < bufferSize; ++i ) {
                //r2 = r1;
                //r1 = rand();

                // TODO: decimator accumulator should be per channel
                // TODO no worky ?

                float nextSample = decimator->processSingle( channelPreMixBuffer[ i ] );

                // correct DC offset and apply dither

                channelPostMixBuffer[ i ] = nextSample;// + DITHER_DC_OFFSET + DITHER_AMPLITUDE * ( float )( r1 - r2 );
                lastSample = nextSample * .25;

                // run the oscillator, note we multiply by .5 and add .5 to make the LFO's bipolar waveforms unipolar

                if ( _hasDownSampleLfo ) {
                    float lfoValue = _downSampleLfo->peek() * .5f + .5f;
                    _tempDownSampleAmount = std::min( _downSampleLfoMax, _downSampleLfoMin + _downSampleLfoRange * lfoValue );
                    cacheValues();
                }
            }
            */

            /* ------ OWN ATTEMPT -----
            // first apply a lowpass filter on the mix buffer to prevent interpolation artefacts

            _lowPassFilters.at( c )->applyFilter( channelPreMixBuffer, bufferSize );
            i = 0;
            float start = 0.f;
            for ( int32 l = std::min( fBufferSize, start + _sampleIncr ); i < l; ++i ) {

                r2 = r1;
                r1 = rand();

                float nextSample = ( channelPreMixBuffer[ i ] * .5 ) + lastSample;

                // correct DC offset and apply dither

                channelPostMixBuffer[ i ] = nextSample + DITHER_DC_OFFSET + DITHER_AMPLITUDE * ( float )( r1 - r2 );
                lastSample = nextSample * .25;

                // run the oscillator, note we multiply by .5 and add .5 to make the LFO's bipolar waveforms unipolar

                if ( _hasDownSampleLfo ) {
                    float lfoValue = _downSampleLfo->peek() * .5f + .5f;
                    _tempDownSampleAmount = std::min( _downSampleLfoMax, _downSampleLfoMin + _downSampleLfoRange * lfoValue );
                    cacheValues();
                    l = std::min( fBufferSize, start + _sampleIncr );
                }
            }
            */

            /* ------ JUCE -----
            // https://github.com/juce-framework/JUCE/blob/master/modules/juce_audio_basics/sources/juce_ResamplingAudioSource.cpp

            // first apply a lowpass filter on the mix buffer to prevent interpolation artefacts

            _lowPassFilters.at( c )->applyFilter( channelPreMixBuffer, bufferSize );

            for ( i = 0; i < bufferSize; ++i ) {
                if ( nextPos == maxBufferPos ) {
                    break;
                }
                const float curSample  = channelPreMixBuffer[ bufferPos ];
                const float nextSample = channelPreMixBuffer[ nextPos ];

                channelPostMixBuffer[ i ] = curSample + subSampleOffset * ( nextSample - curSample );

                subSampleOffset += _ds;

                while ( subSampleOffset >= 1.f )
                {
                    if ( ++bufferPos > maxBufferPos ) {
                        bufferPos = 0;
                    }
                    nextPos = ( bufferPos + 1 ) % bufferSize;
                    subSampleOffset -= 1.f;
                }

                // run the oscillator, note we multiply by .5 and add .5 to make the LFO's bipolar waveforms unipolar

                if ( _hasDownSampleLfo ) {
                    float lfoValue = _downSampleLfo->peek() * .5f + .5f;
                    _tempDownSampleAmount = std::min( _downSampleLfoMax, _downSampleLfoMin + _downSampleLfoRange * lfoValue );
                    cacheValues();
                }
            }
            */

            // JUCE separate
            //_downSamplers.at( c )->process( channelRecordBuffer, _maxRecordBufferSize, channelPostMixBuffer, bufferSize );
        }

        // if down sampling was disabled, omit writing to post mix buffer
        float* out = mustDownSample ? channelPostMixBuffer : channelPreMixBuffer;

        // apply bit crusher
        bitCrusher->process( out, bufferSize );

        // mix the input and processed mix buffers into the output buffer

        for ( i = 0; i < bufferSize; ++i ) {
            // wet mix (e.g. the effected signal)
            channelOutBuffer[ i ] = ( SampleType ) out[ i ] * wetMix;
            // dry mix (e.g. mix in the input signal)
            if ( mixDry ) {
                // before writing to the out buffer we take a snapshot of the current in sample
                // value as VST2 in Ableton Live supplies the same buffer for in and out!
                inSample = channelInBuffer[ i ];
                channelOutBuffer[ i ] += ( inSample * dryMix );
            }
        }
        _lastSamples[ c ] = lastSample;
    }
    // update indices
    _readPointer  = readPointer;
    _writePointer = writePointer;

    _subSampleOffset = subSampleOffset;
    _bufferPos = bufferPos;

    // limit the output signal in case its gets hot (e.g. on heavy bit reduction)
    limiter->process<SampleType>( outBuffer, bufferSize, numOutChannels );
}

template <typename SampleType>
void PluginProcess::prepareMixBuffers( SampleType** inBuffer, int numInChannels, int bufferSize )
{
    // if the record buffer wasn't created yet or the buffer size has changed
    // delete existing buffer and create new one to match properties

    int recordSize = bufferSize * ( int ) ( _maxDownSample / MIN_PLAYBACK_SPEED );
    if ( _recordBuffer == nullptr || _recordBuffer->bufferSize != recordSize ) {
        delete _recordBuffer;
        _recordBuffer = new AudioBuffer( numInChannels, recordSize );
        _maxRecordBufferSize = recordSize;
    }

    // if the pre mix or post mix buffers weren't created yet or the buffer size has changed
    // delete existing buffers and create new ones to match properties

    if ( _preMixBuffer == nullptr || _preMixBuffer->bufferSize != bufferSize ) {
        delete _preMixBuffer;
        _preMixBuffer = new AudioBuffer( numInChannels, bufferSize );
    }

    if ( _postMixBuffer == nullptr || _postMixBuffer->bufferSize != bufferSize ) {
        delete _postMixBuffer;
        _postMixBuffer = new AudioBuffer( numInChannels, bufferSize );
    }
}

}
