//---------------------------------------------------------------------------------------------------------------------
//  robotics MICO plugin
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


#include <mico/robotics/flow/BlockPid.h>
#include <flow/Outpipe.h>
#include <sstream>

namespace mico {
    namespace robotics{
        BlockPid::BlockPid(){
            pid_ = new PID();
            firstTime_ = true;
            createPipe<float>("u");

            createPolicy({ flow::makeInput<float>("reference") ,  flow::makeInput<float>("x") });

            registerCallback(   {"x"}, 
                            &BlockPid::signalInputCallback,
                            this        
            );

            registerCallback({ "reference" },
                &BlockPid::referenceInputCallback,
                this);
        }


        bool BlockPid::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto param = getParamByName(_params, "p"); param) {
                p_ = param.value().asDecimal();
                pid_->kp(p_);
            }
            if (auto param = getParamByName(_params, "i"); param) {
                i_ = param.value().asDecimal();
                pid_->ki(i_);
            }
            if (auto param = getParamByName(_params, "d"); param) {
                d_ = param.value().asDecimal();
                pid_->kd(d_);
            }
            if (auto param = getParamByName(_params, "fc"); param) {
                fc_ = param.value().asDecimal();
                pid_->fc(fc_);
            }
            return true;
        }
        
        std::vector<flow::ConfigParameterDef> BlockPid::parameters(){
            return {
                    {"p", flow::ConfigParameterDef::eParameterType::DECIMAL, 1.0f},
                    {"i", flow::ConfigParameterDef::eParameterType::DECIMAL, 1.0f},
                    {"d", flow::ConfigParameterDef::eParameterType::DECIMAL, 1.0f},
                    {"fc", flow::ConfigParameterDef::eParameterType::DECIMAL, 15.0f}
            };
        }
    
        void BlockPid::signalInputCallback(float _x) {
            if (getPipe("u")->registrations()) {
                if (firstTime_) {
                    t0_ = std::chrono::steady_clock::now();
                    firstTime_ = false;
                }
                else {
                    auto t1 = std::chrono::steady_clock::now();
                    float incT = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0_).count();
                    incT /= 1e6;
                    t0_ = t1;

                    float u = pid_->update(_x, incT);
                    getPipe("u")->flush(u);

                }
            }
        }

        void BlockPid::referenceInputCallback(float _ref) {
            pid_->reference(_ref);
        }
    }
}
