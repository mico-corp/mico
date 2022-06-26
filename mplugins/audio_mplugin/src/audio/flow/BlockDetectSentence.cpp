//---------------------------------------------------------------------------------------------------------------------
//  Cameras wrapper MICO plugin
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------


#include <mico/audio/flow/BlockDetectSentence.h>
#include <flow/Outpipe.h>
#include <algorithm>

namespace mico{
    namespace audio {
        BlockDetectSentence::BlockDetectSentence() {

            createPolicy({ flow::makeInput<std::vector<float>>("input")});
            registerCallback({ "input" }, &BlockDetectSentence::fillBuffer, this);
            createPipe<std::vector<float>>("sentence");
        }

        bool BlockDetectSentence::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto threshold = getParamByName(_params, "SilenceThreshold"); threshold) {
                silenceThreshold_ = threshold.value().asDecimal();
            }
            if (auto samples = getParamByName(_params, "SilenceSamples"); samples) {
                silenceSamples_ = samples.value().asInteger();
            }
            return true;
        }

        BlockDetectSentence::~BlockDetectSentence() {
            
        };


        std::vector<flow::ConfigParameterDef> BlockDetectSentence::parameters() {
            return {
                { "SilenceThreshold", flow::ConfigParameterDef::eParameterType::DECIMAL, 0.02f},
                { "SilenceSamples", flow::ConfigParameterDef::eParameterType::INTEGER, 100}
            };
        }

        void BlockDetectSentence::fillBuffer(std::vector<float> _input) {
            if (!_input.size()) return;

            if (!isRecording_  && !isSilence(_input)) {
                isRecording_ = true;
            }


            if (isRecording_) {
                bufferData_.insert(bufferData_.end(), _input.begin(), _input.end());
                
                if (isSilence(_input)) {
                    samplesCounter_++;
                    if (samplesCounter_ > silenceSamples_) {
                        samplesCounter_ = 0;
                        isRecording_ = false;
                        if(getPipe("sentence")->registrations()) getPipe("sentence")->flush(bufferData_);
                        bufferData_.clear();
                    }
                }
            }
        }


        bool BlockDetectSentence::isSilence(const std::vector<float>& _input) const {
            float level = 0;
            for (auto it = _input.begin(); it != _input.end(); it++) {
                level += std::fabs(*it);
            }
            level /= _input.size();
            return level < silenceThreshold_;
        }

    }
}
