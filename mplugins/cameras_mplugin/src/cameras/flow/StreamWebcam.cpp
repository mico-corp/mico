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


#include <mico/cameras/flow/StreamWebcam.h>
#include <flow/Outpipe.h>

#include <QSpinBox>
#include <QGroupBox>
#include <QLabel>

namespace mico{
    namespace cameras {
        StreamWebcam::StreamWebcam() {
            createPipe<cv::Mat>("Color");
            freqSpinner_ = new QSpinBox;
            freqSpinner_->setValue(30);
            freqSpinner_->setMinimum(1);
            freqSpinner_->setMaximum(200);
        }


        StreamWebcam::~StreamWebcam() {
            if (camera_) {
                camera_->release();
                delete camera_;
            }
        };

        bool StreamWebcam::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (isRunningLoop()) // Cant configure if already running.
                return false;

            if (auto param = getParamByName(_params, "device_id"); param) {
                camera_ = new cv::VideoCapture();

                if (camera_->open(param.value().asInteger(), cv::CAP_DSHOW)) {
                	std::cout << "Using DSHOW Backend" << std::endl;
                }else if (camera_->open(param.value().asInteger(), cv::CAP_GSTREAMER)) {
                	std::cout << "Using GSTREAMER Backend" << std::endl;
        	    }else if (camera_->open(param.value().asInteger(), cv::CAP_V4L)) {
                	std::cout << "Using V4L Backend" << std::endl;
                }else{
			        return false;
		        }

                return camera_->isOpened();
            }

        }

        std::vector<flow::ConfigParameterDef> StreamWebcam::parameters() {
            return {
                {"device_id", flow::ConfigParameterDef::eParameterType::INTEGER, 0}
            };
        }


        QWidget* StreamWebcam::customWidget() {
            auto w = new QWidget();
            auto l = new QHBoxLayout;
            w->setLayout(l);
            l->addWidget(new QLabel("Frequency: "));
            l->addWidget(freqSpinner_);
            return w;
        };

        void StreamWebcam::loopCallback() {
            while (isRunningLoop()) {
                if (auto pipe = getPipe("Color"); pipe->registrations() != 0) {
                    cv::Mat image;
                    if(camera_->grab() && camera_->retrieve(image)) {
                        pipe->flush(image);
                    }
                }
                // Approximate sleep
                float hz = freqSpinner_->value();
                int incT = 1.0f / hz * 1000.0f;
                std::this_thread::sleep_for(std::chrono::milliseconds(incT));
            }
        }
    }
}
