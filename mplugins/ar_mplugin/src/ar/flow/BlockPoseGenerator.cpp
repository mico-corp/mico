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


#include <mico/ar/flow/BlockPoseGenerator.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <opencv2/aruco.hpp>

namespace mico{
    namespace ar {
        BlockPoseGenerator::BlockPoseGenerator(){

            createPipe<Eigen::Matrix4f>("pose");

            createPolicy({  flow::makeInput<float>("x"),
                            flow::makeInput<float>("y"),
                            flow::makeInput<float>("z"),
                            flow::makeInput<float>("roll"),
                            flow::makeInput<float>("pitch"),
                            flow::makeInput<float>("yaw") });

            registerCallback({"x", "y", "z", "roll", "pitch", "yaw"}, 
                [&](flow::DataFlow _data){
                    auto x = _data.get<float>("x");
                    auto y = _data.get<float>("y");
                    auto z = _data.get<float>("z");
                    auto roll = _data.get<float>("roll");
                    auto pitch = _data.get<float>("pitch");
                    auto yaw = _data.get<float>("yaw");

                    Eigen::Matrix4f coordinates = Eigen::Matrix4f::Identity();
                    coordinates.block<3,3>(0,0) =    Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()).matrix()*
                                                    Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()).matrix() *
                                                    Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).matrix();

                    coordinates(0,3) = x;
                    coordinates(1,3) = y;
                    coordinates(2,3) = z;


                    if (getPipe("pose")->registrations()) {
                        getPipe("pose")->flush(coordinates);
                    }
                }
            );

        }

        bool BlockPoseGenerator::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto param = getParamByName(_params, "absolute"); param) {
                isAbsolute_ = param.value().asBool();
            }
            return true;
        }
        
        std::vector<flow::ConfigParameterDef> BlockPoseGenerator::parameters(){
            return {
                {"absolute", flow::ConfigParameterDef::eParameterType::BOOLEAN, 1}
            };
        }

    }
}