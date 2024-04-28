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

#ifdef MICO_IS_RASPBIAN_DEPRECATED

#include <flow/Outpipe.h>
#include <mico/cameras/flow/RaspiCam.h>

namespace mico {
namespace cameras {
RaspiCam::RaspiCam() { createPipe<cv::Mat>("Color"); }

bool RaspiCam::configure(std::vector<flow::ConfigParameterDef> _params) {
  if (isRunningLoop()) // Cant configure if already running.
    return false;

  return camera_.open();
}

std::vector<flow::ConfigParameterDef> RaspiCam::parameters() { return {{}}; }

void RaspiCam::loopCallback() {
  while (isRunningLoop()) {
    if (auto pipe = getPipe("Color"); pipe->registrations() != 0) {
      cv::Mat image;
      if (camera_.grab()) {
        camera_.retrieve(image);
        pipe->flush(image);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
} // namespace cameras
} // namespace mico

#endif