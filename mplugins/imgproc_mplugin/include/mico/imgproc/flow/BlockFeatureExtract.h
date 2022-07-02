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



#ifndef MICO_FLOW_BLOCKS_BLOCKFEATUREEXTRACT_H_
#define MICO_FLOW_BLOCKS_BLOCKFEATUREEXTRACT_H_

#include <flow/Block.h>

#include <opencv2/opencv.hpp>
#include <mutex>

#include <opencv2/features2d.hpp>

class QTableWidget;

namespace mico{
    namespace imgproc{
        /// Mico block to extract 2D features in images
        /// @ingroup  mico_imgproc
        ///
        /// @image html blocks/imgproc/imgproc_block_feature_detector width=480px
        ///
        /// __Inputs__:
        ///     * input: image as cv::Mat for detecting features.
        ///
        /// __Outputs__:
        ///     * features: vector of features detected by the detector as std::vector<cv::Keypoint> type.
        ///     * descriptors: descriptors computed by the detector in cv::Mat format.
        ///     * debug: debug image as cv::Mat.
        ///
        /// __parameters__:
        ///     * Features: type of detector to be used
        ///
        class BlockFeatureExtract:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Feature extractor";}        

            /// Base constructor. Initializes the neural network.
            BlockFeatureExtract();

            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir() / "img_proc" / "block_feature_extractor.svg").string();
            }
            
            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            /// Returns a brief description of the block
            std::string description() const override {return    "Extract features from the input image and outputs a vector with them."
                                                                "   - Inputs: \n"
                                                                "   - Outputs: \n";};

                
        private:
            void policyCallback(cv::Mat _img);

        private:
            std::mutex dataLock_;
            cv::Ptr<cv::Feature2D> detector_;
        };
    }
}



#endif