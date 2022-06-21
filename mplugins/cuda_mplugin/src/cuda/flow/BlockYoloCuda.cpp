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


#include <mico/cuda/flow/BlockYoloCuda.h>
#include <flow/Outpipe.h>

#include <cmath>

#include <sstream>

#include <opencv2/core/cuda.hpp>

namespace mico{
    namespace cuda{
        BlockYoloCuda::BlockYoloCuda(){

            try{
                net_ = cv::dnn::readNetFromDarknet(
                    (flow::Persistency::resourceDir() / "ml"/"yolov3-tiny.cfg").string(), 
                    (flow::Persistency::resourceDir() / "ml"/"yolov3-tiny.weights").string());
                if(hasCuda()){
                    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
                } else {
                    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                }
                outputs_ = net_.getUnconnectedOutLayersNames();
            }catch(std::exception &_e){
                std::cerr << "[WARNING] Failed to read YOLO parameters, cant use YOLO ML block" << std::endl;
            }

            
            createPipe<cv::Mat>("image");
            createPipe<std::vector<ml::Detection>>("detections");

            createPolicy({  flow::makeInput<cv::Mat>("input") });

            registerCallback(   {"input"}, 
                                &BlockYoloCuda::policyCallback,
                                this
            );
        }

        void BlockYoloCuda::policyCallback(cv::Mat _image) {

            if (getPipe("image")->registrations() || getPipe("detections")->registrations()) {
                cv::Mat frame = _image.clone();
                cv::Mat blob;
                std::vector<cv::Mat> result;
                try {
                    cv::dnn::blobFromImage(frame, blob, 0.00392, cv::Size(320, 320), cv::Scalar(), true, false, CV_32F);
                    net_.setInput(blob);
                    net_.forward(result, outputs_);

                }
                catch (std::exception& _e) {
                    std::cout << _e.what() << std::endl;
                }

                auto detections = parseDetections(frame, result);
                if (getPipe("image")->registrations()) {
                    getPipe("image")->flush(frame);
                }
                if (getPipe("detections")->registrations()) {
                    getPipe("detections")->flush(detections);
                }

            }
        }

        std::vector<ml::Detection> BlockYoloCuda::parseDetections(cv::Mat &_frame, const std::vector<cv::Mat> &_detections){
            std::vector<ml::Detection> outDetections;
            constexpr float CONFIDENCE_THRESHOLD = 0.0f;
            constexpr float NMS_THRESHOLD = 0.4f;
            constexpr int NUM_CLASSES = 80;
            const cv::Scalar colors[] = {
                {0, 255, 255},
                {255, 255, 0},
                {0, 255, 0},
                {255, 0, 0}
            };
            const auto NUM_COLORS = sizeof(colors)/sizeof(colors[0]);

            std::vector<int> indices[NUM_CLASSES];
            std::vector<cv::Rect> boxes[NUM_CLASSES];
            std::vector<float> scores[NUM_CLASSES];

            for (auto& output : _detections) {
                const auto num_boxes = output.rows;

                for (int i = 0; i < num_boxes; i++) {
                    auto x = output.at<float>(i, 0) * _frame.cols;
                    auto y = output.at<float>(i, 1) * _frame.rows;
                    auto width = output.at<float>(i, 2) * _frame.cols;
                    auto height = output.at<float>(i, 3) * _frame.rows;
                    cv::Rect rect(x - width/2, y - height/2, width, height);

                    for (int c = 0; c < NUM_CLASSES; c++) {
                        auto confidence = *output.ptr<float>(i, 5 + c);
                        if (confidence >= CONFIDENCE_THRESHOLD) {
                            boxes[c].push_back(rect);
                            scores[c].push_back(confidence);
                        }
                    }
                }
            }

            for (int c = 0; c < NUM_CLASSES; c++)
                cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD, indices[c]);
            
            for (int c= 0; c < NUM_CLASSES; c++)
            {
                for (size_t i = 0; i < indices[c].size(); ++i)
                {
                    const auto color = colors[c % NUM_COLORS];

                    auto idx = indices[c][i];
                    const auto& rect = boxes[c][idx];
                    cv::rectangle(_frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width, rect.y + rect.height), color, 3);
                    std::ostringstream label_ss;
                    label_ss << /*class_names[c]*/ "label " << c << ": " << std::fixed << std::setprecision(2) << scores[c][idx];
                    auto label = label_ss.str();
                    
                    int baseline;
                    auto label_bg_sz = cv::getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
                    cv::rectangle(_frame, cv::Point(rect.x, rect.y - label_bg_sz.height - baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y), color, cv::FILLED);
                    cv::putText(_frame, label.c_str(), cv::Point(rect.x, rect.y - baseline - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
                    
                    outDetections.push_back({c, rect, _frame(rect & cv::Rect(0,0,_frame.cols, _frame.rows))});
                }
            }
            return outDetections;
        }

        bool BlockYoloCuda::hasCuda() {
            cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());
            int cuda_devices_number = cv::cuda::getCudaEnabledDeviceCount();
            cv::cuda::DeviceInfo _deviceInfo;
            return cuda_devices_number!= 0 && _deviceInfo.isCompatible();
        }
    }
}