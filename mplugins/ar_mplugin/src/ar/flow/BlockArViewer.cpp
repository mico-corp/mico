//---------------------------------------------------------------------------------------------------------------------
//  AR MICO plugin
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


#include <mico/ar/flow/BlockArViewer.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

namespace mico{
    namespace ar {
        BlockArViewer::BlockArViewer(){
            createPolicy({  flow::makeInput<Eigen::Matrix4f>("coordinates"),
                            flow::makeInput<cv::Mat>("image") });

            registerCallback({ "coordinates", "image" },
                &BlockArViewer::policyCallback,
                this
            );

        }
        
        BlockArViewer::~BlockArViewer() {
            if (widget_) {
                widget_->hide();
                delete widget_;
            }
        }

        
        bool BlockArViewer::configure(std::vector<flow::ConfigParameterDef> _params){
            if (widget_) {
                widget_->hide();
                delete widget_;
            }
            widget_ = new VisualizerGlWidget();
            widget_->show();

            return true;
        }

        void BlockArViewer::policyCallback(Eigen::Matrix4f _coordinates, cv::Mat _image) {
            if (!widget_) return;
            widget_->updatePose(_coordinates);
            widget_->updateBackgroundImage(_image);
        }
    }
}