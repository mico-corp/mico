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
#include <mico/cameras/flow/SingleImageFlusher.h>

#include <QPushButton>

namespace mico {
namespace cameras {
SingleImageFlusher::SingleImageFlusher() {
  createPipe<cv::Mat>("Image");

  centralWidget_ = new QPushButton("Flush image");
  QObject::connect(centralWidget_, &QPushButton::clicked, [&]() {
    if (loadedImage_.rows != 0 && getPipe("Image")->registrations() != 0) {
      getPipe("Image")->flush(loadedImage_);
    }
  });
}

bool SingleImageFlusher::configure(
    std::vector<flow::ConfigParameterDef> _params) {
  if (auto param = getParamByName(_params, "image_path"); param) {
    try {
      loadedImage_ = cv::imread(param.value().asPath().string());
    } catch (std::exception &e) {
      std::cout << e.what() << std::endl;
    }
    return loadedImage_.rows != 0;
  }
  return false;
}

std::vector<flow::ConfigParameterDef> SingleImageFlusher::parameters() {
  return {{"image_path", flow::ConfigParameterDef::eParameterType::PATH,
           fs::path("Browse Path")}};
}
QWidget *SingleImageFlusher::customWidget() { return centralWidget_; }
} // namespace cameras
} // namespace mico
