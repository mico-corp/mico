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
        /// Mico block uses  YOLO DNN to generate an stream of detections.
        /// @ingroup  mico_ml
        class BlockKernelConvolution:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Convolution kernel";}        

            /// Base constructor. Initializes the neural network.
            BlockKernelConvolution();
            
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
            void setItemValidator();
            void normalizeKernel();
        
        private:
            std::vector< cv::String > outputs_;
            bool idle_ = true;

            QTableWidget *table_;
            std::mutex dataLock_;
            cv::Mat kernel_;
        };
    }
}



#endif