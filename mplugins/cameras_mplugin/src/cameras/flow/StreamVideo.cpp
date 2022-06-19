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


#include <mico/cameras/flow/StreamVideo.h>
#include <flow/Outpipe.h>

#include <QSpinBox>
#include <QGroupBox>
#include <QLabel>

namespace mico{
    namespace cameras {
        StreamVideo::StreamVideo() {
            createPipe<cv::Mat>("Color");
            createPipe<int>("width");
            createPipe<int>("height");
        }


        StreamVideo::~StreamVideo() {
            if (isRunningLoop()) this->stop();

            if (videoFile_) {
                videoFile_->release();
                delete videoFile_;
            }
            std::unique_lock lck(safeDeletion_);
            cv_.wait_for(lck, std::chrono::milliseconds(100));
        };

        bool StreamVideo::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (isRunningLoop()) // Cant configure if already running.
                return false;

            if (auto param = getParamByName(_params, "video_file"); param) {
                videoFile_ = new cv::VideoCapture();

                if (videoFile_->open(param.value().asPath().string())) {
                }else{
			        return false;
		        }

                return videoFile_->isOpened();
            }
            return false;
        }

        std::vector<flow::ConfigParameterDef> StreamVideo::parameters() {
            return {
                {"video_file", flow::ConfigParameterDef::eParameterType::PATH, 0}
            };
        }

        void StreamVideo::loopCallback() {
            while(!videoFile_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            while (isRunningLoop()) {
                if (auto pipe = getPipe("Color"); pipe->registrations() != 0) {
                    cv::Mat image;
                    if(videoFile_->grab() && videoFile_->retrieve(image)) {
                        pipe->flush(image);
                        getPipe("width")->flush(image.cols);
                        getPipe("height")->flush(image.rows);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            }
            cv_.notify_all();
        }
    }
}
