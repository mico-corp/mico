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

#ifndef MICO_FLOW_BLOCKS_BLOCKFACEFEATURES_H_
#define MICO_FLOW_BLOCKS_BLOCKFACEFEATURES_H_

#include <flow/Block.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <mico/ml/Detection.h>

#include <dlib/clustering.h>
#include <dlib/dnn.h>
#include <dlib/opencv.h>
#include <dlib/string.h>

template <template <int, template <typename> class, int, typename> class block,
          int N, template <typename> class BN, typename SUBNET>
using residual = dlib::add_prev1<block<N, BN, 1, dlib::tag1<SUBNET>>>;

template <template <int, template <typename> class, int, typename> class block,
          int N, template <typename> class BN, typename SUBNET>
using residual_down = dlib::add_prev2<dlib::avg_pool<
    2, 2, 2, 2, dlib::skip1<dlib::tag2<block<N, BN, 2, dlib::tag1<SUBNET>>>>>>;

template <int N, template <typename> class BN, int stride, typename SUBNET>
using block =
    BN<dlib::con<N, 3, 3, 1, 1,
                 dlib::relu<BN<dlib::con<N, 3, 3, stride, stride, SUBNET>>>>>;

template <int N, typename SUBNET>
using ares = dlib::relu<residual<block, N, dlib::affine, SUBNET>>;
template <int N, typename SUBNET>
using ares_down = dlib::relu<residual_down<block, N, dlib::affine, SUBNET>>;

template <typename SUBNET> using alevel0 = ares_down<256, SUBNET>;
template <typename SUBNET>
using alevel1 = ares<256, ares<256, ares_down<256, SUBNET>>>;
template <typename SUBNET>
using alevel2 = ares<128, ares<128, ares_down<128, SUBNET>>>;
template <typename SUBNET>
using alevel3 = ares<64, ares<64, ares<64, ares_down<64, SUBNET>>>>;
template <typename SUBNET> using alevel4 = ares<32, ares<32, ares<32, SUBNET>>>;

using anet_type = dlib::loss_metric<dlib::fc_no_bias<
    128,
    dlib::avg_pool_everything<
        alevel0<alevel1<alevel2<alevel3<alevel4<dlib::max_pool<
            3, 3, 2, 2,
            dlib::relu<dlib::affine<dlib::con<
                32, 7, 7, 2, 2, dlib::input_rgb_image_sized<150>>>>>>>>>>>>>;

namespace mico {
namespace ml {
/// Mico block uses DLIB face feature detector.
/// @ingroup  mico_ml
///
/// @image html blocks/ml width=480px
///
/// __Inputs__:
///     * image: image as cv::Mat to compute the feature vector
///
/// __Outputs__:
///     * FeatureVector: 128D Feature vector generated from input image.
///
class BlockFaceFeatures : public flow::Block {
public:
  /// Get name of block
  std::string name() const override { return "Face Features"; }

  /// Base constructor. Initializes the neural network.
  BlockFaceFeatures();

  /// Retreive icon of block
  std::string icon() const override {
    return (flow::Persistency::resourceDir() / "ml" / "block_face_features.png")
        .string();
  }

  /// Return if the block is configurable.
  bool isConfigurable() override { return false; };

  /// Returns a brief description of the block
  std::string description() const override {
    return "Compute feature vector of face"
           "   - Inputs: Image of a face\n"
           "   - Outputs: 128D feature vector\n";
  };

private:
  void policyCallback(cv::Mat _image);

private:
  anet_type featureDetector_;
};
} // namespace ml
} // namespace mico

#endif