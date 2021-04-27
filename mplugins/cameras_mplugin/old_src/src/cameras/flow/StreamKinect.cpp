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


#include <mico/cameras/flow/StreamKinect.h>
#include <flow/Outpipe.h>

namespace mico{

        StreamKinect::StreamKinect(){
            createPipe("Color", "image");
            createPipe("Depth", "image");
            createPipe("Cloud", "cloud");
        }

        bool StreamKinect::configure(std::vector<flow::ConfigParameterDef> _params) {
            if(isRunningLoop() || hasInitCamera_) // Cant configure if already running.
                return true;

            cjson::Json jParams = {};
            for(auto &p:_params){
                if(p.first == "calibFile"){
                    jParams["calibFile"] = p.second.c_str();
                }
            }

            hasInitCamera_ = camera_.init(jParams);
            
            return hasInitCamera_;
        }
        
        std::vector<flow::ConfigParameterDef> StreamKinect::parameters(){
            return {
                {"calibFile", flow::ConfigParameterDef::eParameterType::STRING}
            };
        }

        void StreamKinect::loopCallback() {
            if(!hasInitCamera_){
                std::cout << "Cant init Kinect camera if not configured first" << std::endl;
                return;
            }
                
            for(unsigned i = 0; i < 10; i++){
                camera_.grab(); // 666 Grab some images to remove trash initial ones
            }

            while(isRunningLoop()){
                cv::Mat left, right, depth;
                pcl::PointCloud<pcl::PointXYZRGBNormal> colorNormalCloud;
                camera_.grab();
                if(getPipe("Color")->registrations() !=0 ){
                    camera_.rgb(left, right);
                    getPipe("Color")->flush(left);     
                }
                if(getPipe("Depth")->registrations() !=0 ){
                    camera_.depth(depth);
                    getPipe("Depth")->flush(depth);
                }
                if(getPipe("Cloud")->registrations() !=0 ){
                    camera_.cloud(colorNormalCloud);
                    getPipe("Cloud")->flush(colorNormalCloud.makeShared());
                }
            }      
        }
}