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


#include <mico/audio/flow/BlockSpectrogram.h>
#include <flow/Outpipe.h>
#include <algorithm>

namespace mico{
    namespace audio {
        BlockSpectrogram::BlockSpectrogram() {

            createPolicy({ flow::makeInput<std::vector<float>>("input")});
            registerCallback({ "input" }, &BlockSpectrogram::inputCallback, this);
            createPipe<cv::Mat>("spectrogram");
        }

        bool BlockSpectrogram::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto threshold = getParamByName(_params, "SampleSize"); threshold) {
                if (rawDft_ != nullptr) {
                    delete[] rawDft_;
                    rawDft_ = nullptr;
                }

                sampleSize_ = threshold.value().asInteger();
                rawDft_ = new fftw_complex[size_t(sampleSize_ / 2) + 1];
            }
            if (auto samples = getParamByName(_params, "StepSize"); samples) {
                stepSize_ = samples.value().asInteger();
            }
            if (auto samples = getParamByName(_params, "SizeSpectrogram"); samples) {
                sizeSpectrogram_ = samples.value().asInteger();
            }
            if (auto samples = getParamByName(_params, "CutoffFrequency"); samples) {
                maxFrequency_ = samples.value().asInteger();
            }

            spectrogram_ = cv::Mat(maxFrequency_, sizeSpectrogram_, CV_32FC1, cv::Scalar(0.0f));

            return true;
        }

        BlockSpectrogram::~BlockSpectrogram() {
            if (rawDft_ != nullptr) delete[] rawDft_;
            
        };


        std::vector<flow::ConfigParameterDef> BlockSpectrogram::parameters() {
            return {
                { "SampleSize", flow::ConfigParameterDef::eParameterType::INTEGER, 2048},
                { "StepSize", flow::ConfigParameterDef::eParameterType::INTEGER, 512},
                { "SizeSpectrogram", flow::ConfigParameterDef::eParameterType::INTEGER, 100},
                { "CutoffFrequency", flow::ConfigParameterDef::eParameterType::INTEGER, 100}
            };        
        }

        void BlockSpectrogram::inputCallback(std::vector<float> _input) {
            if (!_input.size()) return;

            bufferData_.insert(bufferData_.end(), _input.begin(), _input.end());
            if (bufferData_.size() > sampleSize_) {
                computeDFT(bufferData_);
                bufferData_.erase(bufferData_.begin(), bufferData_.begin() + stepSize_);
            }
            if(getPipe("spectrogram")->registrations()) getPipe("spectrogram")->flush(spectrogram_);
            
        }

        void BlockSpectrogram::computeDFT(const std::vector<float> & _signal){
            std::vector<double> signal(_signal.begin(), _signal.begin()+sampleSize_);
            fftw_plan plan = fftw_plan_dft_r2c_1d(sampleSize_, signal.data(), rawDft_, NULL);
            fftw_execute(plan);

            std::vector<float> output;
            output.resize(maxFrequency_);
            for (int i = 0; i < maxFrequency_; i++) {
                spectrogram_.at<float>(maxFrequency_ - i - 1, currentSample_) = 2.0 / (double)(sampleSize_)*fabs(rawDft_[i][0]);
            }
            fftw_destroy_plan(plan);
            currentSample_++;
            currentSample_ = currentSample_ % sizeSpectrogram_;
        }

    }
}
