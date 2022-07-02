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



#ifndef MICO_FLOW_BLOCKS_STREAMERS_STREAMVIDEO_H_
#define MICO_FLOW_BLOCKS_STREAMERS_STREAMVIDEO_H_

#include <flow/Block.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <condition_variable>

class QSpinBox;

namespace mico{
    namespace cameras{
        /// Mico block that opens video files to stream
        /// @ingroup  mico_cameras
        ///
        /// @image html blocks/cameras/cameras_block_streamer_webcam.png width=480px
        ///
        /// __Outputs__:
        ///     * Image: image as cv::Mat obtained from webcam
        ///     * width: width of output images.
        ///     * height: height of output images.
        ///
        /// __parameters__:
        ///     * Frequency: speed of output stream
        ///     * device_id: id of the camera, depend on operative system. Typically starts from 0.
        ///
        class StreamVideo:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Streamer Video";}     
            
            /// Retreive icon of block    
            QIcon icon() const override { 
                return QIcon((flow::Persistency::resourceDir() / "cameras"/"webcam_icon.svg").string().c_str());
            }
            
            /// Base constructor
            StreamVideo();

            /// Base destructor
            ~StreamVideo();
            
            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;
            
            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Method to check if the block has auto running callback
            bool hasRunLoop() const override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Streamer block that reads from usb ready cameras "
                                                                "connected to the computer and streams its images.\n"
                                                                "   - Outputs: \n";};

        protected:
            void loopCallback() override;

        private:
            cv::VideoCapture * videoFile_ = nullptr;
            std::mutex safeDeletion_;
            std::condition_variable cv_;
        };
    }
}



#endif