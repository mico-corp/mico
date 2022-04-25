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

            createPolicy({  flow::makeInput<cv::Mat>("input")  });

            registerCallback(   {"input"}, 
                                [&](flow::DataFlow _data){
                                    if(!idle_ || !isConfigured_)
                                        return;

                                    idle_ = false;
                                    if(getPipe("result")->registrations() != 0 || getPipe("detections")->registrations() != 0){
                                        cv::Mat frame = _data.get<cv::Mat>("input").clone();
                                        cv::Mat frame_gray;
                                        cv::cvtColor( frame, frame_gray, cv::COLOR_BGR2GRAY );
                                        cv::equalizeHist( frame_gray, frame_gray );
                                        //-- Detect faces
                                        std::vector<cv::Rect> faces;
                                        if (!face_cascade.empty()) {
                                            face_cascade.detectMultiScale( frame_gray, faces );   
                                            std::vector< Detection> detections;
                                            for ( size_t i = 0; i < faces.size(); i++ ) {
                                                cv::Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
                                                cv::rectangle( frame, faces[i], cv::Scalar( 0, 255, 0 ), 4 );
                                                
                                                detections.push_back({ 0, faces[i], frame(faces[i])});
                                            }
                                            if(getPipe("result")->registrations() != 0 ) getPipe("result")->flush(frame);
                                            if(getPipe("detections")->registrations() != 0) 
                                                getPipe("detections")->flush(detections);
                                            
                                        }

                                    }
                                    idle_ = true;
                                }
            );
        }

        
        bool BlockHaarCascade::configure(std::vector<flow::ConfigParameterDef> _params) {
            while (!idle_) {    // 666 This blocks the GUI, but it is a quick fix needed to release a working version.
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
            isConfigured_ = false;
            if(auto detector = getParamByName(_params, "Detector"); detector){
                if( !face_cascade.load( detectors[detector.value().asString()])) {
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
    }
}   