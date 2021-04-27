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


#include <mico/ar/flow/BlockMesh.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

namespace mico{
    namespace ar {
        BlockMesh::BlockMesh(){
            scene_ = Scene3d::get();

            createPolicy({  flow::makeInput<Eigen::Matrix4f>("coordinates")});

            registerCallback({ "coordinates" },
                [&](flow::DataFlow _data) {
                    if (!idle_) return;
                    
                    idle_ = false;
                    Eigen::Matrix4f coordinates = _data.get<Eigen::Matrix4f>("coordinates");
                    if(mesh_){
                        mesh_->transform(coordinates);
                    }
                    idle_ = true;
                }
            );

        }
        
        
        bool BlockMesh::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto param = getParamByName(_params, "mesh_path"); param) {
                mesh_ = std::make_shared<Mesh>();
                if (mesh_->loadMesh(param.value().asString())) {
                    scene_->addMesh(mesh_);
                    return true;
                }
            }
            return false;
        }
        
        std::vector<flow::ConfigParameterDef> BlockMesh::parameters(){
            return {
                {"mesh_path", flow::ConfigParameterDef::eParameterType::STRING, std::string("")}
            };
        }
    }
}