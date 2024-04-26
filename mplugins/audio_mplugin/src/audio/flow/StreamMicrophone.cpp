//-----------------------------------------------------------------------------
//  Cameras wrapper MICO plugin
//-----------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//-----------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to
//  deal in the Software without restriction, including without limitation the
//  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
//  sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//  IN THE SOFTWARE.
//-----------------------------------------------------------------------------

#include <flow/Outpipe.h>
#include <mico/audio/flow/StreamerMicrophone.h>

using namespace std::experimental;

namespace mico {
namespace audio {
StreamerMicrophone::StreamerMicrophone() {
  if (get_default_audio_input_device().has_value()) {
    unsigned nChannels =
        get_default_audio_input_device().value().get_num_input_channels();
    for (unsigned i = 0; i < nChannels; i++) {
      createPipe<std::vector<float>>("Channel " + std::to_string(i));
    }
  }
}

StreamerMicrophone::~StreamerMicrophone() {
  if (device_)
    device_->stop();
};

bool StreamerMicrophone::configure(
    std::vector<flow::ConfigParameterDef> _params) {
  if (device_)
    device_->stop();

  if (auto gain = getParamByName(_params, "gain"); gain.has_value()) {
    gain_ = gain.value().asDecimal();
  }

  // createPipe<cv::Mat>("Color");
  if (get_default_audio_input_device().has_value()) {
    device_ = get_default_audio_input_device();

    device_->connect([this](audio_device &_dev,
                            audio_device_io<float> &io) mutable noexcept {
      this->microphoneCallback(_dev, io);
    });

    device_->start();
  }
  return true;
}

std::vector<flow::ConfigParameterDef> StreamerMicrophone::parameters() {
  return {{"gain", flow::ConfigParameterDef::eParameterType::DECIMAL, 1.0f}};
}

void StreamerMicrophone::microphoneCallback(audio_device &,
                                            audio_device_io<float> &io) {
  if (!io.input_buffer.has_value())
    return;

  auto &in = *io.input_buffer;

  frames_.resize(in.size_channels());

  for (int channel = 0; channel < in.size_channels(); ++channel) {
    frames_[channel].resize(in.size_frames());
    memcpy(&frames_[channel][0], in.data(), sizeof(float) * in.size_frames());
    if (gain_ != 1.0f) {
      std::transform(frames_[channel].begin(), frames_[channel].end(),
                     frames_[channel].begin(),
                     [&](double element) { return element *= gain_; });
    }

    auto pipe = getPipe("Channel " + std::to_string(channel));
    if (pipe && pipe->registrations())
      pipe->flush(frames_[channel]);
  }
}

} // namespace audio
} // namespace mico
