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



#ifndef MICO_CAMERAS_BLOCKCALIBRATIONMONOCULAR_H_
#define MICO_CAMERAS_BLOCKCALIBRATIONMONOCULAR_H_

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
        class BlockCalibrationMonocular :public flow::Block {
        public:
            /// Get name of block
            std::string name() const override { return "Calibrate Monocular"; }

            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir() / "cameras" / "camera_calibration_icon.svg").string();
            }

            /// Base constructor
            BlockCalibrationMonocular();

            /// Base destructor
            ~BlockCalibrationMonocular();

            bool isConfigurable() { return false; };

            /// Returns a brief description of the block
            std::string description() const override {
                return      "Calibrate camera from stream of images using pinhole model.\n"
                            "   - Input: \n";
            };

            QWidget* customWidget() override;

        private:
            void startReset();

            void computeStatistics(const std::vector<cv::Point2f>& _points, int& _size, int& _x, int& _y);
            void checkRefineAndAdd(cv::Mat& _image, std::vector<cv::Point2f>& _points);

            void calibrate();
	    void policyCallback(cv::Mat  _image);
	    
        private:
            std::mutex imgLock_;

            int vSize_ = 6;
            int hSize_ = 8;
            int mmSize_ = 20;

            QLabel* imageView_ = nullptr;
            QTimer* imageRefresher_ = nullptr;
            cv::Mat lastImage_;

            QProgressBar* progressX_ = nullptr;
            QProgressBar* progressY_ = nullptr;
            QProgressBar* progressSize_ = nullptr;

            std::vector<uint8_t> qualityX_;
            std::vector<uint8_t> qualityY_;
            std::vector<uint8_t> qualityScale_;
            
            std::vector<std::vector<cv::Point2f>> calibPoints_;

        };
    }
}



#endif
