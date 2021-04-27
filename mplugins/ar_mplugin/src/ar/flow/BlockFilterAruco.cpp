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


#include <mico/ar/flow/BlockFilterAruco.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <opencv2/aruco.hpp>

namespace mico{
    namespace ar {
        BlockFilterAruco::BlockFilterAruco(){
            
            createPipe<Eigen::Matrix4f>("coordinates");

            createPolicy({  flow::makeInput<std::map<int, Eigen::Matrix4f>>("all_coordinates") });

            registerCallback({"all_coordinates"}, 
                [&](flow::DataFlow _data){
                    if(!idle_) return;
                    idle_ = false;
                    auto options = _data.get<std::map<int, Eigen::Matrix4f>>("all_coordinates");

                    if(options.find(id_) != options.end()){
                        if (getPipe("coordinates")->registrations()) {
                            getPipe("coordinates")->flush(options[id_]);
                        }
                    }
                                   
                    idle_ = true;
                }
            );

        }

        bool BlockFilterAruco::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto param = getParamByName(_params, "id"); param) {
                id_ = param.value().asInteger();
            }
            return true;
        }
        
        std::vector<flow::ConfigParameterDef> BlockFilterAruco::parameters(){
            return {
                {"id", flow::ConfigParameterDef::eParameterType::INTEGER, 1}
            };
        }

    }
}