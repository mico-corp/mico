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

            registerCallback({"vector"},
                                    [&](flow::DataFlow _data) {
                                        auto v = _data.get<std::vector<float>>("vector");
                                        float res = 0;
                                        for (const auto& e : v) {
                                            res += e * e;
                                        }
                                        res = sqrt(res);

                                        getPipe("norm")->flush(res);
                                    }
            );
        }


        BlockVectorElementWiseOperator::BlockVectorElementWiseOperator() {
            createPipe<std::vector<float>>("result");

            createPolicy({ flow::makeInput<std::vector<float>>("v1"), flow::makeInput<std::vector<float>>("v2") });

            registerCallback({"v1", "v2"},
                                    [&](flow::DataFlow _data) {
                                        auto v1 = _data.get<std::vector<float>>("v1");
                                        auto v2 = _data.get<std::vector<float>>("v2");
                                        if (getPipe("result")->registrations() && v1.size() == v2.size()) {
                                            for (unsigned i = 0; i < v1.size(); i++) {
                                                v1[i] = fn_(v1[i], v2[i]);
                                            }
                                            getPipe("result")->flush(v1);
                                        }

                                    }
            );
        }



        bool BlockVectorElementWiseOperator::configure(std::vector<flow::ConfigParameterDef> _params) {
            std::string param = _params[0].asString();
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