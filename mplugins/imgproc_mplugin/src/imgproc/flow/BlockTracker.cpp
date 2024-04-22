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


#include <mico/imgproc/flow/BlockTracker.h>
#include <flow/Outpipe.h>
#include <opencv2/tracking.hpp>


namespace mico{
    namespace imgproc{
        BlockTracker::BlockTracker(){

            createPipe<float>("x");
            createPipe<float>("y");
            createPipe<bool>("isTracking");
            createPipe<cv::Mat>("debug");

            createPolicy({ flow::makeInput<cv::Mat>("input"), flow::makeInput<cvRect>("initBB") });

            registerCallback({ "initBB" },
                &BlockTracker::callbackInitBB,
                this
            );

            registerCallback(   {"input"}, 
                                &BlockTracker::callbackInputImage,
                                this        
            );
        }
        
        bool BlockTracker::configure(std::vector<flow::ConfigParameterDef> _params) {
            std::lock_guard<std::mutex> lock(dataLock_);

            if(auto param = getParamByName(_params, "Algorithm"); param){
                lastTrackerName_ = param.value().selectedOption();
                createTracker(lastTrackerName_);
            }

            return true;
        }
    
        std::vector<flow::ConfigParameterDef> BlockTracker::parameters(){
            std::vector<std::string> features = {"Mil", "KCF","Goturn", "CSRT"};

            return {
                {"Algorithm", flow::ConfigParameterDef::eParameterType::OPTIONS, features}
            };
        }

        QWidget* BlockTracker::customWidget(){
            auto* w = new QWidget;
            auto* l = new QVBoxLayout;
            w->setLayout(l);

            auto *initBt = new QPushButton("Init box");
            l->addWidget(initBt);
            QObject::connect(initBt, &QPushButton::clicked, [&]() {
                std::lock_guard<std::mutex> lock(dataLock_);
                isInit_ = false;
                if (lastImage_.rows != 0) {
                    bbox_ = cv::selectROI(lastImage_);
                    tracker_->init(lastImage_, bbox_);
                    isInit_ = true;
                }
            });

            auto *stopBt = new QPushButton("Stop tracker");
            l->addWidget(stopBt);
            QObject::connect(stopBt, &QPushButton::clicked, [&]() {
                std::lock_guard<std::mutex> lock(dataLock_);
                isInit_ = false;
            });

            return w;
        }

        void BlockTracker::createTracker(const std::string& _trackerName) {
            if (_trackerName == "Mil") {
                tracker_ = cv::TrackerMIL::create();
            }
            else if (_trackerName == "KCF") {
                tracker_ = cv::TrackerKCF::create();
            }
            else if (_trackerName == "Goturn") {
                tracker_ = cv::TrackerGOTURN::create();
            }
            else if (_trackerName == "CSRT") {
                tracker_ = cv::TrackerCSRT::create();
            }
        }

        void BlockTracker::callbackInitBB(cvRect _bb) {
            std::lock_guard<std::mutex> lock(dataLock_);
            if (isInit_) return;

            isInit_ = false;
            if (lastImage_.rows != 0) {
                createTracker(lastTrackerName_);
                bbox_ = _bb;
                if (bbox_.width > 10 && bbox_.height > 10) {
                    tracker_->init(lastImage_, bbox_);
                    isInit_ = true;
                }
            }
        }

        void BlockTracker::callbackInputImage(cv::Mat  _img) {
            if (isInit_)
                return;

            if (getPipe("x")->registrations() || getPipe("y")->registrations() || getPipe("debug")->registrations()) {
                if (!tracker_) {
                    return;
                }

                bool ok = false;
                cv::Mat frame = _img.clone();
                {
                    std::lock_guard<std::mutex> lock(dataLock_);
                    lastImage_ = frame.clone();
                }
                if (isInit_) {
                    // Update the tracking result
                    {
                        bbox_ = bbox_ & cvRect(0, 0, frame.cols - 1, frame.rows - 1);
                        std::lock_guard<std::mutex> lock(dataLock_);
                        ok = tracker_->update(frame, bbox_);
                    }

                    if (!ok) isInit_ = false;

                    if (getPipe("x")->registrations()) getPipe("x")->flush(bbox_.x + bbox_.width / 2.0f);
                    if (getPipe("y")->registrations()) getPipe("y")->flush(bbox_.y + bbox_.height / 2.0f);
                }
                if (getPipe("isTracking")->registrations()) getPipe("isTracking")->flush(ok);


                if (getPipe("debug")->registrations()) {
                    if (ok) // Tracking success : Draw the tracked object
                        rectangle(frame, bbox_, cv::Scalar(255, 0, 0), 2, 1);
                    else // Tracking failure detected.
                        putText(frame, "Tracking failure detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);

                    getPipe("debug")->flush(frame);
                }
            }
        }
    }
}