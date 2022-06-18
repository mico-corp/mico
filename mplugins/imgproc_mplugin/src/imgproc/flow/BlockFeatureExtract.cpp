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


#include <mico/imgproc/flow/BlockFeatureExtract.h>
#include <flow/Outpipe.h>
//#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>

namespace mico{
    namespace imgproc{
        BlockFeatureExtract::BlockFeatureExtract(){

            createPipe<std::vector<cv::KeyPoint>>("features");
            createPipe<cv::Mat>("descriptors");
            createPipe<cv::Mat>("debug");

            createPolicy({  flow::makeInput<cv::Mat>("input") });

            registerCallback<cv::Mat>(   {"input"},
                                [&](cv::Mat _img){
                                    if(getPipe("features")->registrations() || getPipe("descriptors")->registrations() || getPipe("debug")->registrations()){
                                        if (!detector_)
                                            return;

                                        cv::Mat frame = _img.clone(); 
                                        cv::Mat result;
                                        if (frame.channels() == 1) {
                                            cv::cvtColor(frame,frame,cv::COLOR_BGR2GRAY);
                                        } 

                                        std::vector<cv::KeyPoint> kp;
                                        cv::Mat desc;
                                        
                                        dataLock_.lock();
                                        if (getPipe("descriptors")->registrations()) {
                                            detector_->detectAndCompute(frame, cv::Mat(), kp, desc);
                                        } else {
                                            detector_->detect(frame, kp);
                                        }
                                        dataLock_.unlock();

                                        if (getPipe("features")->registrations()) getPipe("features")->flush(kp);
                                        if (getPipe("descriptors")->registrations())getPipe("descriptors")->flush(desc);
                                        if (getPipe("debug")->registrations()) {
                                            cv::Mat debug;
                                            cv::drawKeypoints(frame, kp, debug);
                                            getPipe("debug")->flush(debug);
                                        }
                                    }
                                }
            );
        }

        
        bool BlockFeatureExtract::configure(std::vector<flow::ConfigParameterDef> _params) {
            std::lock_guard<std::mutex> lock(dataLock_);

            if(auto param = getParamByName(_params, "Features"); param){
                auto featureName = param.value().selectedOption();
                if (featureName == "ORB") {
                    detector_ = cv::ORB::create();
                } else if (featureName == "AKAZE"){
                    detector_ = cv::AKAZE::create();
                } else {
                    return false;
                }
            }

            return true;
        }
    
        std::vector<flow::ConfigParameterDef> BlockFeatureExtract::parameters(){
            std::vector<std::string> features = {"ORB", "AKAZE"};

            return {
                {"Features", flow::ConfigParameterDef::eParameterType::OPTIONS, features}
            };
        }


    }
}