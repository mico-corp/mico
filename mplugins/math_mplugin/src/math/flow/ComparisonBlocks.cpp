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


#include <mico/math/flow/ComparisonBlocks.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <cmath>

#include <sstream>


namespace mico {
    namespace math{
        ComparisonBlock::ComparisonBlock() {
            createPipe<bool>("result");

            createPolicy({ flow::makeInput<float>("A"),
                            flow::makeInput<float>("B") });

            registerCallback({ "A", "B" },
                [&](flow::DataFlow _data) {
                    auto A = _data.get<float>("A");
                    auto B = _data.get<float>("B");

                    getPipe("result")->flush(fn_(A, B));
                }
            );
        }


        bool ComparisonBlock::configure(std::vector<flow::ConfigParameterDef> _params) {
            std::string param = _params[0].asString();
            if (param == "A==B") {
                fn_ = [](float a, float b) {return a == b; };
            }
            else if (param == "A>B") {
                fn_ = [](float a, float b) {return a > b; };
            }
            else if (param == "A<B") {
                fn_ = [](float a, float b) {return a < b; };
            }
            else if (param == "A>=B") {
                fn_ = [](float a, float b) {return a >= b; };
            }
            else if (param == "A<=B") {
                fn_ = [](float a, float b) {return a <= b; };
            }
            else {
                fn_ = [](float a, float b) {return false; };
            }

            return true;
        }

        std::vector<flow::ConfigParameterDef> ComparisonBlock::parameters() {
            std::vector<std::string> options = { "A==B", "A>B", "A<B", "A>=B", "A<=B" };
            return {
                { "Operation", flow::ConfigParameterDef::eParameterType::OPTIONS,  options }
            };
        }

    }
        
}