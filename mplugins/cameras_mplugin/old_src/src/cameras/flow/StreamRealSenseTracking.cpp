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

#ifdef ENABLE_LIBREALSENSE_V2

#include <mico/cameras/flow/StreamRealSenseTracking.h>

namespace mico{

    StreamRealSenseTracking::StreamRealSenseTracking(){
        createPipe("Fisheye", "image");
        createPipe("Pose", "mat44");
    }

    bool StreamRealSenseTracking::configure(std::vector<flow::ConfigParameterDef> _params) {
        if(isRunningLoop() || hasInitCamera_) // Cant configure if already running.
            return true;

        cjson::Json jParams = {};
        for(auto &p:_params){
            if(p.first == "devide_id"){
                jParams["deviceId"] = atoi(p.second.c_str());
            }
        }

        hasInitCamera_ = camera_.init(jParams);
        
        return hasInitCamera_;
    }
    
    std::vector<flow::ConfigParameterDef> StreamRealSenseTracking::parameters(){
        return {
            {"devide_id", flow::ConfigParameterDef::eParameterType::INTEGER} 
        };
    }

    void StreamRealSenseTracking::loopCallback() {
        if(!hasInitCamera_){
            std::cout << "Cant init Realsense camera if not configured first" << std::endl;
            return;
        }
            
        for(unsigned i = 0; i < 10; i++){
            camera_.grab(); // 666 Grab some images to remove trash initial ones
        }

        while(isRunningLoop()){
            cv::Mat fisheye;
            Eigen::Matrix4f pose;
            
            camera_.grab();

            if(getPipe("Fisheye")->registrations() !=0 ){
                camera_.fisheye(fisheye);
                cv::cvtColor(fisheye, fisheye, cv::COLOR_GRAY2RGB);
                getPipe("Fisheye")->flush(fisheye);     
            }
            if(getPipe("Pose")->registrations() !=0 ){
                camera_.pose(pose);
                getPipe("Pose")->flush(pose);
            }
        }      
    }

    QWidget * StreamRealSenseTracking::customWidget() {
        QGroupBox * box = new QGroupBox;
        
        QHBoxLayout * layout = new QHBoxLayout;
        QPushButton *button = new QPushButton("Reset Pose");
        layout->addWidget(button);
        
        box->setLayout(layout);

        QWidget::connect(button, &QPushButton::clicked, [this](){
            if (camera_.reset())
                std::cout << "Tracking camera reset sucesfully \n";
            
        });

        return box;
    }
}

#endif