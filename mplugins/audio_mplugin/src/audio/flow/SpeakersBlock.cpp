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


#include <mico/audio/flow/SpeakersBlock.h>
#include <flow/Outpipe.h>

using namespace std::experimental;

namespace mico{
    namespace audio {
        SpeakersBlock::SpeakersBlock() {
            if (get_default_audio_output_device().has_value()) {
                nChannels_ = get_default_audio_output_device().value().get_num_output_channels();
                bufferData_.resize(nChannels_);

                for (unsigned c = 0; c < nChannels_; c++) {
                    createPolicy({ flow::makeInput<std::vector<float>>("input" + std::to_string(c)) });
                    std::function<void(std::vector<float>)> fn = std::bind(&SpeakersBlock::fillBuffer, this, std::placeholders::_1, c);
                    registerCallback<std::vector<float>>({ "input" + std::to_string(c) }, fn );

                }


            }
        }

        bool SpeakersBlock::configure([[maybe_unused]]std::vector<flow::ConfigParameterDef> _params) {
            if (device_) device_->stop();

            device_ = get_default_audio_output_device();
            device_->connect([this](audio_device&_dev, audio_device_io<float>& io) mutable noexcept{
                this->audioCallback(_dev, io);
                });

            device_->start();
            
            return true;
        }

        SpeakersBlock::~SpeakersBlock() {
            if(device_) device_->stop();
        }

        void SpeakersBlock::fillBuffer(const std::vector<float>& _input, unsigned _channel) {
            dataLock_.lock();
            bufferData_[_channel].insert(bufferData_[_channel].end(), _input.begin(), _input.end());
            dataLock_.unlock();
        }

        void SpeakersBlock::audioCallback(audio_device&, audio_device_io<float>& io) {
            if (!io.output_buffer.has_value())
                return;

            auto& out = *io.output_buffer;
            for (unsigned channel = 0; channel < nChannels_; ++channel) {
                auto& buffer = bufferData_[channel];
                if (buffer.size() > out.size_frames()) {
                    if (out.is_contiguous()) {
                        dataLock_.lock();
                        memcpy(out.data(), &buffer[0], sizeof(float) * out.size_frames());
                        buffer.erase(buffer.begin(), buffer.begin() + out.size_frames());
                        dataLock_.unlock();
                    }
                }
            }
        }


    }
}
