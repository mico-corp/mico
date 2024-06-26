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

#ifndef MICO_CUDA_BLOCKS_BLOCKYOLOCUDA_H_
#define MICO_CUDA_BLOCKS_BLOCKYOLOCUDA_H_

#include <flow/Block.h>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/opencv.hpp>

#include <mico/ml/Detection.h>

namespace mico {
namespace cuda {
/// Mico block uses  YOLO DNN to generate an stream of detections.
/// @ingroup  mico_ml
class BlockYoloCuda : public flow::Block {
public:
  /// Get name of block
  std::string name() const override { return "Yolo DNN [CUDA]"; }

  /// Base constructor. Initializes the neural network.
  BlockYoloCuda();

  /// Retreive icon of block
  std::string icon() const override {
    return (flow::Persistency::resourceDir() / "cuda" / "block_yolo_cuda.svg")
        .string();
  }

  /// Return if the block is configurable.
  bool isConfigurable() override { return false; };

  /// Returns a brief description of the block
  std::string description() const override {
    return "Detect objects Haars cascade algorithm"
           "   - Inputs: \n"
           "   - Outputs: \n";
  };

private:
  void policyCallback(cv::Mat _image);
  std::vector<ml::Detection>
  parseDetections(cv::Mat &_frame, const std::vector<cv::Mat> &_detections);

  bool hasCuda();

private:
  cv::dnn::Net net_;
  std::vector<cv::String> outputs_;
};
} // namespace cuda
} // namespace mico

#endif