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



#ifndef MICO_IMGPROC_FLOW_BLOCKTRACKER
#define MICO_IMGPROC_FLOW_BLOCKTRACKER

#include <flow/Block.h>

#include <opencv2/opencv.hpp>
#include <mutex>

#include <opencv2/features2d.hpp>

#include <QPushButton>

namespace mico{
    namespace imgproc{
        /// Mico block uses  YOLO DNN to generate an stream of detections.
        /// @ingroup  mico_ml
        class BlockTracker:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Tracker";}        

            /// Base constructor. Initializes the neural network.
            BlockTracker();

            ~BlockTracker();

            /// Retreive icon of block    
            QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "img_proc" / "block_tracker.svg").string().c_str());
            }
            
            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            QWidget* customWidget() override;

            /// Returns a brief description of the block
            std::string description() const override {return    "Track an object within an image."
                                                                "   - Inputs: Image \n"
                                                                "   - Outputs: x, y, debug image\n";};

                
        private:
            bool idle_ = true;
            std::mutex dataLock_;
            cv::Ptr<cv::Tracker> tracker_ = nullptr;
            QPushButton* initBt_ = nullptr;
            bool isInit_ = false;
            cv::Rect bbox_;
            cv::Mat lastImage_;

            std::mutex safeDeletion_;
            std::condition_variable cv_;

        };
    }
}



#endif