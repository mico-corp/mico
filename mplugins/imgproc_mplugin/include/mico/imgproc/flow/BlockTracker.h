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

#ifndef MICO_IMGPROC_FLOW_BLOCKTRACKER
#define MICO_IMGPROC_FLOW_BLOCKTRACKER

#include <flow/Block.h>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include <condition_variable>
#include <mutex>

#include <QPushButton>

namespace mico {
namespace imgproc {
/// Mico block that implements an object tracker for 2d images.
/// @ingroup  mico_imgproc
///
/// @image html blocks/imgproc/imgproc_block_object_tracker width=480px
///
/// __Inputs__:
///     * input: image as cv::Mat to apply the convolution.
///     * initBB: initial bounding box as cv::Rect to start tracker.
///
/// __Outputs__:
///     * x: X coordinate of the center of the tracked object as float.
///     * y: Y coordinate of the center of the tracked object as float.
///     * isTracking: boolean to indicate if the tracker has lost or not the
///     object
///     * debug: debug image as cv::Mat to be displayed.
///
/// __parameters__:
///     * Algorithm: available object trackers.
///
class BlockTracker : public flow::Block {
public:
  /// Get name of block
  std::string name() const override { return "Tracker"; }

  /// Base constructor. Initializes the neural network.
  BlockTracker();

  /// Retreive icon of block
  std::string icon() const override {
    return (flow::Persistency::resourceDir() / "img_proc" / "block_tracker.svg")
        .string();
  }

  /// Return if the block is configurable.
  bool isConfigurable() override { return true; };

  /// Configure block with given parameters.
  bool configure(std::vector<flow::ConfigParameterDef> _params) override;

  /// Get list of parameters of the block
  std::vector<flow::ConfigParameterDef> parameters() override;

  QWidget *customWidget() override;

  /// Returns a brief description of the block
  std::string description() const override {
    return "Track an object within an image."
           "   - Inputs: Image \n"
           "   - Outputs: x, y, debug image\n";
  };

private:
  typedef cv::Rect cvRect;

  void createTracker(const std::string &_trackerName);
  void callbackInitBB(cvRect _bb);
  void callbackInputImage(cv::Mat _img);

private:
  std::mutex dataLock_;
  cv::Ptr<cv::Tracker> tracker_ = nullptr;
  std::string lastTrackerName_ =
      ""; // App crashes when reinitializing an already runned tracker, so keep
          // track of name to recreate it every initialization.

  bool isInit_ = false;
  cvRect bbox_;
  cv::Mat lastImage_;

  std::mutex safeDeletion_;
  std::condition_variable cv_;
};
} // namespace imgproc
} // namespace mico

#endif