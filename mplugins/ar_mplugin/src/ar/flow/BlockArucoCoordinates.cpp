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


#include <mico/ar/flow/BlockArucoCoordinates.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <opencv2/aruco.hpp>

namespace mico{
    namespace ar {
        BlockArucoCoordinates::BlockArucoCoordinates(){
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);;

            createPipe<Eigen::Matrix4f>("coordinates");
            createPipe<std::map<int, Eigen::Matrix4f>>("all_coordinates");
            createPipe<cv::Mat>("output_image");

            createPolicy({  flow::makeInput<cv::Mat>("image") });

            registerCallback({"image"}, 
                [&](flow::DataFlow _data){
                    if(!idle_) return;
                    idle_ = false;
                    auto image = _data.get<cv::Mat>("image").clone();
                    if(!image.rows){
                        idle_ = true;
                        return ;
                    } 

                    cv::rotate(image, image, cv::ROTATE_180);


                    std::vector<int> ids;
                    std::vector<std::vector<cv::Point2f>> corners;
                    cv::aruco::detectMarkers(image, dictionary_, corners, ids);
                    // if at least one marker detected
                    if (ids.size() > 0) {
                        cv::aruco::drawDetectedMarkers(image, corners, ids);

                        if (isCalibrated_) {
                            std::vector<cv::Vec3d> rvecs, tvecs;
                            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix_, distCoeffs_, rvecs, tvecs);


                            if (id_ == 0 && getPipe("all_coordinates")->registrations()) {
                                std::map<int, Eigen::Matrix4f> all;
                                // draw axis for each marker
                                for (int id = 0; id < ids.size(); id++) {
                                    cv::Mat R;
                                    cv::Rodrigues(rvecs[id], R);

                                    Eigen::Matrix4f coordinates = Eigen::Matrix4f::Identity();
                                    for (unsigned i = 0; i < 3; i++) {
                                        for (unsigned j = 0; j < 3; j++) {
                                            coordinates(i, j) = R.at<double>(i, j);
                                        }
                                        coordinates(i, 3) = tvecs[id](i);
                                    }
                                    all[ids[id]] = coordinates;
                                }
                                getPipe("all_coordinates")->flush(all);
                            }
                            else {
                                // draw axis for each marker
                                for (int id = 0; id < ids.size(); id++) {
                                    //cv::aruco::drawAxis(image, cameraMatrix_, distCoeffs_, rvecs[id], tvecs[id], 0.1);
                                    if (ids[id] == id_) { // use as coordinate system
                                        cv::Mat R;
                                        cv::Rodrigues(rvecs[id],R);

                                        Eigen::Matrix4f coordinates = Eigen::Matrix4f::Identity();
                                        for (unsigned i = 0; i < 3; i++) {
                                            for (unsigned j = 0; j < 3; j++) {
                                                coordinates(i, j) = R.at<double>(i, j);
                                            }
                                            coordinates(i, 3) = tvecs[id](i);
                                        }
                                        if (getPipe("coordinates")->registrations()) {
                                            getPipe("coordinates")->flush(coordinates);
                                        }
                                    }
                                }
                            }

                        }
                    }

                    
                    if (getPipe("output_image")->registrations()) {
                        getPipe("output_image")->flush(image);
                    }

                    idle_ = true;
                }
            );

        }

        bool BlockArucoCoordinates::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto param = getParamByName(_params, "id"); param) {
                id_ = param.value().asInteger();
            }

            if (auto param = getParamByName(_params, "calibration_file"); param) {
                std::string paramFile = param.value().asString();
                
                cv::FileStorage fs;
                try {
                    fs.open(paramFile, cv::FileStorage::READ);
                }
                catch (cv::Exception& e) {
                    return false;
                }
                
                if (fs.isOpened()) {
                    fs["Matrix"] >> cameraMatrix_;
                    fs["DistCoeffs"] >> distCoeffs_;
                
                    isCalibrated_ = true;
                }
                else {
                    isCalibrated_ = false;
                }
            }

            return isCalibrated_;
        }
        
        std::vector<flow::ConfigParameterDef> BlockArucoCoordinates::parameters(){
            return {
                {"id", flow::ConfigParameterDef::eParameterType::INTEGER, 1},
                {"calibration_file", flow::ConfigParameterDef::eParameterType::STRING, std::string("")}
            };
        }
    }

}