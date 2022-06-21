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


#include <mico/cameras/flow/BlockImageResize.h>
#include <flow/Outpipe.h>

#include <QSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QProgressBar>
#include <QTimer>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>

namespace mico {
    namespace cameras {
        BlockImageResize::BlockImageResize() {
            createPipe<cv::Mat>("out");
            createPolicy({ flow::makeInput<cv::Mat>("in") });

            registerCallback({ "in" },
                &BlockImageResize::policyCallback,
                this
            );

        }


        std::vector<flow::ConfigParameterDef> BlockImageResize::parameters() {
            return {
                {"width", flow::ConfigParameterDef::eParameterType::INTEGER, 640},
                {"height", flow::ConfigParameterDef::eParameterType::INTEGER, 480}
            };
        }

        bool BlockImageResize::configure(std::vector<flow::ConfigParameterDef> _params) {
            
            if (auto param = getParamByName(_params, "width"); param)
                width_ = param.value().asInteger();
            else return false;
            
            if (auto param = getParamByName(_params, "height"); param) 
                height_ = param.value().asInteger();
            else return false;

            return true;
        }

        void BlockImageResize::policyCallback(cv::Mat _image) {
            cv::Mat image = _image.clone();
            if (image.rows != 0) {
                if (getPipe("out")->registrations()) {
                    cv::resize(image, image, cv::Size(width_, height_));
                    getPipe("out")->flush(image);
                }
            }
        }

    }
}
