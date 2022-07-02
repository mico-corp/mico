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


#include <mico/math/flow/VectorBlocks.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <cmath>

#include <sstream>


namespace mico{
    namespace math{



        BlockVectorNorm::BlockVectorNorm() {
            createPipe<float>("norm");

            createPolicy({ flow::makeInput<std::vector<float>>("vector") });


            std::function<void(std::vector<float>)> fn = [&](std::vector<float> _v) {
                float res = 0;
                for (const auto& e : _v) {
                    res += e * e;
                }
                res = sqrt(res);

                getPipe("norm")->flush(res);
            };
            registerCallback({"vector"},fn);
        }


        BlockVectorElementWiseOperator::BlockVectorElementWiseOperator() {
            createPipe<std::vector<float>>("result");

            createPolicy({ flow::makeInput<std::vector<float>>("v1"), flow::makeInput<std::vector<float>>("v2") });

            std::function<void(std::vector<float>, std::vector<float>)> fn = [&](std::vector<float> _v1, std::vector<float> _v2) {
                if (getPipe("result")->registrations() && _v1.size() == _v2.size()) {
                    for (unsigned i = 0; i < _v1.size(); i++) {
                        _v1[i] = fn_(_v1[i], _v2[i]);
                    }
                    getPipe("result")->flush(_v1);
                }

            };
            registerCallback({"v1", "v2"},fn);
        }



        bool BlockVectorElementWiseOperator::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (_params.size() != 1) return false;
            if (_params[0].type_ != flow::ConfigParameterDef::eParameterType::OPTIONS) return false;

            std::string param = _params[0].selectedOption();
            if (param == "+") {
                fn_ = [](float a, float b) {return a + b; };
            }
            else if (param == "-") {
                fn_ = [](float a, float b) {return a - b; };
            }
            else if (param == "*") {
                fn_ = [](float a, float b) {return a * b; };
            }
            else if (param == "/") {
                fn_ = [](float a, float b) {return a / b; };
            }
            else {
                return false;
            }

            return true;
        }

        std::vector<flow::ConfigParameterDef> BlockVectorElementWiseOperator::parameters() {
            std::vector<std::string> options = { "+", "-", "*", "/", };
            return {
                { "Operation", flow::ConfigParameterDef::eParameterType::OPTIONS,  options }
            };
        }
    }
}