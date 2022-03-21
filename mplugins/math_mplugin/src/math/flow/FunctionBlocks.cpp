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


#include <mico/math/flow/FunctionBlocks.h>
#include <flow/Outpipe.h>

#include <sstream>

#include <cmath>


namespace mico{
    namespace math{
        //-------------------------------------------------------------------------------------------------------------
        // Block Timer
        //-------------------------------------------------------------------------------------------------------------
        BlockTimer::BlockTimer(){
                createPipe<float>("timer");
            }

            bool BlockTimer::configure(std::vector<flow::ConfigParameterDef> _params) {
                if (auto resolution = getParamByName(_params, "resolution"); resolution)
                    resolution_ = resolution.value().asDecimal();

                return true;
            }
            
            std::vector<flow::ConfigParameterDef> BlockTimer::parameters(){
                return {
                    {"resolution", flow::ConfigParameterDef::eParameterType::DECIMAL, 0.001f}
                };
            }

            void BlockTimer::loopCallback() {
                auto t0  = std::chrono::steady_clock::now();
                auto t0r  = std::chrono::steady_clock::now();
                while(isRunningLoop()){
                    auto t1  = std::chrono::steady_clock::now();
                    float incTr = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0r).count();
                    if(incTr/1e6 > resolution_){
                        float incT = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
                        t0r = t1;
                        if(auto pipe = getPipe("timer"); pipe->registrations() !=0 ){
                            pipe->flush(float(incT/1e6));     
                        }
                        
                    }
                }         
            }


        //-------------------------------------------------------------------------------------------------------------
        // Block Sine
        //-------------------------------------------------------------------------------------------------------------
        BlockSine::BlockSine(){
            createPipe<float>("result");

            createPolicy({flow::makeInput<float>("time")});

            registerCallback({"time"}, [&](flow::DataFlow _data){
                                        auto t = _data.get<float>("time"); 
                                                
                                        float result = amplitude_*sin(freq_*t + phase_);
                                        getPipe("result")->flush(result);
                                    });
        }

        bool BlockSine::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto amplitude = getParamByName(_params, "amplitude"); amplitude)
                amplitude_ = amplitude.value().asDecimal();

            if (auto frequency = getParamByName(_params, "frequency"); frequency)
                freq_ = frequency.value().asDecimal();

            if (auto phase = getParamByName(_params, "phase"); phase)
                phase_ = phase.value().asDecimal();

            return true;
        }
        
        std::vector<flow::ConfigParameterDef> BlockSine::parameters(){
            return {
                {"amplitude", flow::ConfigParameterDef::eParameterType::DECIMAL, 1.0f}, 
                {"frequency", flow::ConfigParameterDef::eParameterType::DECIMAL, 1.0f}, 
                {"phase", flow::ConfigParameterDef::eParameterType::DECIMAL, 0.0f}
            };
        }

        //-------------------------------------------------------------------------------------------------------------
        // Block Cosine
        //-------------------------------------------------------------------------------------------------------------
        BlockCosine::BlockCosine() {
            createPipe<float>("result");

            createPolicy({ flow::makeInput<float>("time") });

            registerCallback({ "time" }, [&](flow::DataFlow _data) {
                auto t = _data.get<float>("time");

                float result = amplitude_ * cos(freq_ * t + phase_);
                getPipe("result")->flush(result);
                });
        }

        bool BlockCosine::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto amplitude = getParamByName(_params, "amplitude"); amplitude)
                amplitude_ = amplitude.value().asDecimal();

            if (auto frequency = getParamByName(_params, "frequency"); frequency)
                freq_ = frequency.value().asDecimal();

            if (auto phase = getParamByName(_params, "phase"); phase)
                phase_ = phase.value().asDecimal();

            return true;
        }

        std::vector<flow::ConfigParameterDef> BlockCosine::parameters() {
            return {
                {"amplitude", flow::ConfigParameterDef::eParameterType::DECIMAL, 1.0f},
                {"frequency", flow::ConfigParameterDef::eParameterType::DECIMAL, 1.0f},
                {"phase", flow::ConfigParameterDef::eParameterType::DECIMAL, 0.0f}
            };
        }


        //-------------------------------------------------------------------------------------------------------------
        // Block Linear Map
        //-------------------------------------------------------------------------------------------------------------
        BlockLinearMap::BlockLinearMap() {
            createPipe<float>("output");

            createPolicy({ flow::makeInput<float>("input") });

            registerCallback({ "input" }, [&](flow::DataFlow _data) {
                auto in = _data.get<float>("input");

                float result = (in - minInput_) / (maxInput_ - minInput_) * (maxOutput_ - minOutput_) + minOutput_;
                getPipe("output")->flush(result);
                });
        }

        bool BlockLinearMap::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto minInput = getParamByName(_params, "min input"); minInput)
                minInput_ = minInput.value().asDecimal();

            if (auto maxInput = getParamByName(_params, "max input"); maxInput)
                maxInput_ = maxInput.value().asDecimal();

            if (auto minOutput = getParamByName(_params, "min output"); minOutput)
                minOutput_ = minOutput.value().asDecimal();

            if (auto maxOutput = getParamByName(_params, "max input"); maxOutput)
                maxOutput_ = maxOutput.value().asDecimal();

            return true;
        }

        std::vector<flow::ConfigParameterDef> BlockLinearMap::parameters() {
            return {
                {"min input", flow::ConfigParameterDef::eParameterType::DECIMAL, 0.0f},
                {"max input", flow::ConfigParameterDef::eParameterType::DECIMAL, 1024.0f},
                {"min output", flow::ConfigParameterDef::eParameterType::DECIMAL, 0.0f},
                {"max output", flow::ConfigParameterDef::eParameterType::DECIMAL, 180.0f}
            };
        }


    }
}