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



#ifndef MICO_CAMERAS_BLOCKIMAGERESIZE_H_
#define MICO_CAMERAS_BLOCKIMAGERESIZE_H_

#include <flow/Block.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>

class QLabel;
class QTimer;
class QProgressBar;

namespace mico {
    namespace cameras {
        /// Mico block that opens USB camera devices and flush images out on a stream.
        /// @ingroup  mico_cameras
        ///
        /// @image html blocks/cameras/cameras_block_calibrate_monocular.png width=480px
        ///
        /// __Inputs__:
        ///     * Image: image as cv::Mat to be used to generate a calibration file.
        ///
        /// __parameters__:
        ///     * Horizontal Size: Number of horizontal divisions of the calibration pattern.
        ///     * Vertical Size: Number of vertical divisions of the calibration pattern.
        ///     * Square size: size of squares in calibration pattern in millimeters.
        ///
        class BlockImageResize :public flow::Block {
        public:
            /// Get name of block
            std::string name() const override { return "Image Resize"; }

            /// Retreive icon of block    
            QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "cameras" / "icon_resize.png").string().c_str());
            }

            /// Base constructor
            BlockImageResize();

            bool isConfigurable() { return true; };
            
            std::vector<flow::ConfigParameterDef> parameters();

            bool configure(std::vector<flow::ConfigParameterDef> _params);

            /// Returns a brief description of the block
            std::string description() const override {
                return      "Resize images of a stream.\n"
                            "   - Input: Image \n"
                            "   - Output: Resized image \n";
            };

        private:
            bool idle_ = true;
            int width_ = 640;
            int height_ = 480;

            
        };
    }
}



#endif