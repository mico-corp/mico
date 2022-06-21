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


#include <mico/ml/flow/FaceDetectors.h>
#include <mico/ml/Detection.h>
#include <flow/Outpipe.h>

#include <cmath>

#include <sstream>

namespace mico{
    namespace ml{
        BlockHaarCascade::BlockHaarCascade(){
            createPipe<cv::Mat>("result");
            createPipe<std::vector<Detection>>("detections");
            createPipe<int>("n_detections");

            createPolicy({  flow::makeInput<cv::Mat>("input")  });

            registerCallback({ "input" },
                &BlockHaarCascade::policyCallback,
                this
            );
        }

        BlockHaarCascade::~BlockHaarCascade() {
            std::lock_guard<std::mutex> lock(safeDestroy_);
        }

        bool BlockHaarCascade::configure(std::vector<flow::ConfigParameterDef> _params) {
            
            isConfigured_ = false;
            if(auto detector = getParamByName(_params, "Detector"); detector){
                if( !face_cascade.load( detectors[detector.value().selectedOption()])) {
                    return false;
                }
            }
            isConfigured_ = true;
            return true;
        }
        
        std::vector<flow::ConfigParameterDef> BlockHaarCascade::parameters(){
            return {
                {   "Detector", 
                    flow::ConfigParameterDef::eParameterType::OPTIONS, 
                    std::vector<std::string>{"Face detector",  "Body detector",  "Upperbody detector"} 
                }  
            };
        }

        void BlockHaarCascade::policyCallback(cv::Mat _image) {
            if (!isConfigured_)
                return;

            std::lock_guard<std::mutex> lock(safeDestroy_);

            if (getPipe("result")->registrations() != 0 || getPipe("detections")->registrations() != 0 || getPipe("n_detections")->registrations() != 0) {
                cv::Mat frame = _image.clone();
                cv::Mat frame_gray;
                cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
                cv::equalizeHist(frame_gray, frame_gray);
                //-- Detect faces
                std::vector<cv::Rect> faces;
                if (!face_cascade.empty()) {
                    face_cascade.detectMultiScale(frame_gray, faces);
                    std::vector< Detection> detections;
                    for (size_t i = 0; i < faces.size(); i++) {
                        cv::Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);
                        cv::rectangle(frame, faces[i], cv::Scalar(0, 255, 0), 4);

                        detections.push_back({ 0, faces[i], frame(faces[i]) });
                    }
                    if (getPipe("result")->registrations() != 0) getPipe("result")->flush(frame);
                    if (getPipe("detections")->registrations() != 0) getPipe("detections")->flush(detections);
                    if (getPipe("n_detections")->registrations() != 0) getPipe("n_detections")->flush((int)detections.size());

                }

            }
        }
    }
}   