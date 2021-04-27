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


#include <mico/math/flow/ConstStreamer.h>
#include <flow/Outpipe.h>
#include <QSpinBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>

#include <sstream>


namespace mico{
    namespace math{
        ConstStreamer::ConstStreamer(){
            createPipe<float>("value");
        }

        bool ConstStreamer::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto value = getParamByName(_params, "value"); value)
                value_ = value.value().asDecimal();
            if (auto rate = getParamByName(_params, "rate"); rate)
                rate_ = rate.value().asInteger();

            return true;
        }
        
        std::vector<flow::ConfigParameterDef> ConstStreamer::parameters(){
            return {
                {"value", flow::ConfigParameterDef::eParameterType::DECIMAL, 0.0f}, 
                {"rate", flow::ConfigParameterDef::eParameterType::INTEGER, 1}
            };
        }

        QWidget * ConstStreamer::customWidget(){
            return nullptr;
        }

        void ConstStreamer::loopCallback() {
            auto t0  = std::chrono::steady_clock::now();
            while(isRunningLoop()){
                auto t1  = std::chrono::steady_clock::now();
                float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
                if(incT / 1000 > 1/rate_){
                    t0 = t1;
                    if(auto pipe = getPipe("value"); pipe->registrations() !=0 ){
                        pipe->flush(value_);     
                    }
                    
                }
            }         
        }
    }
}