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


#include <mico/simulation/flow/BlockFirstOrderSystem.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <cmath>

#include <sstream>


namespace mico{
    namespace simulation {
        BlockFirstOrderSystem::BlockFirstOrderSystem(){
            createPipe<float>("output");

            createPolicy({  flow::makeInput<float>("time"),
                            flow::makeInput<float>("input") });


            std::function<void(float)> cb1 = [&](float _t) {
                simulate(_t);
                getPipe("output")->flush(y_);
            };
            registerCallback({"time"},  cb1 );

            std::function<void(float)> cb2 = [&](float _u) { u_ = _u; };
            registerCallback({"input"},  cb2);
        }

        bool BlockFirstOrderSystem::configure(std::vector<flow::ConfigParameterDef> _params) {
            if(isRunningLoop()) // Cant configure if already running.
                return false;            

            if(auto param = getParamByName(_params, "K"); param){
                k_ = param.value().asDecimal();
            }

            if (auto param = getParamByName(_params, "Tau"); param) {
                tau_ = param.value().asDecimal();
            }

            if (auto param = getParamByName(_params, "x0"); param) {
                y0_ = param.value().asDecimal();
            }
            isFirstTime_ = true;

            return true;
        }
        
        std::vector<flow::ConfigParameterDef> BlockFirstOrderSystem::parameters(){
            return {
                {"K", flow::ConfigParameterDef::eParameterType::DECIMAL, 1.0f},
                {"Tau", flow::ConfigParameterDef::eParameterType::DECIMAL, 1.0f},
                {"x0", flow::ConfigParameterDef::eParameterType::DECIMAL, 0.0f}
            };
        }

        void BlockFirstOrderSystem::simulate(float _t){
            if (isFirstTime_) {
                prevTime_ = _t;
                isFirstTime_ = false;
                y_ = y0_;
                yp_ = k_ / tau_ * u_ + y_ / tau_;
            } else {
                float incT = _t - prevTime_;
                prevTime_ = _t;
                y_ += incT * yp_;
                yp_ = k_ / tau_ * u_ - y_ / tau_;
            }
            
        }
    }
}