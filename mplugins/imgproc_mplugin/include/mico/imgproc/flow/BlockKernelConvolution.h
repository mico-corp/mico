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



#ifndef MICO_FLOW_BLOCKS_BLOCKKERNELCONVOLUTION_H_
#define MICO_FLOW_BLOCKS_BLOCKKERNELCONVOLUTION_H_

#include <flow/Block.h>

#include <opencv2/opencv.hpp>
#include <mutex>

class QTableWidget;

namespace mico{
    namespace imgproc{
        /// Mico block that performs a convolution on an image with the given kernel
        /// @ingroup  mico_imgproc
        ///
        /// @image html blocks/imgproc/imgproc_block_kernel_convolution width=480px
        ///
        /// __Inputs__:
        ///     * input: image as cv::Mat to apply the convolution.
        ///
        /// __Outputs__:
        ///     * output: image as cv::Mat with applied the convolution.
        ///
        /// __parameters__:
        ///     * Table to write the kernel 
        ///
        class BlockKernelConvolution:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Convolution kernel";}        

            /// Base constructor. Initializes the neural network.
            BlockKernelConvolution();

            /// Retreive icon of block    
            QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "img_proc" / "block_convolution.svg").string().c_str());
            }
            
            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Applies kernel to an image"
                                                                "   - Inputs: \n"
                                                                "   - Outputs: \n";};

            /// Virtual method to tell the interface if the visual block is resizable or not.
            bool resizable() const override { return true; }
            
            QWidget* customWidget() override;
        
        private:
            void policyCallback(cv::Mat _img);
            void setItemValidator();
            void normalizeKernel();
        
        private:
            std::vector< cv::String > outputs_;

            QTableWidget *table_;
            std::mutex dataLock_;
            cv::Mat kernel_;
        };
    }
}



#endif