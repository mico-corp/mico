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


#include <mico/math/flow/ArithmeticBlocks.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <cmath>

#include <sstream>


namespace mico{
    namespace math{
        BlockSum::BlockSum(){
            createPipe<float>("result");

            createPolicy({  flow::makeInput<float>("in_1"),
                            flow::makeInput<float>("in_2") });

            registerCallback({"in_1", "in_2"}, 
                                    [&](flow::DataFlow _data){
                                        auto v1 = _data.get<float>("in_1");
                                        auto v2 = _data.get<float>("in_2"); 
                                             
                                        getPipe("result")->flush(v1+v2);
                                    }
            );
        }


        BlockSubstract::BlockSubstract(){
            createPipe<float>("result");

            createPolicy({  flow::makeInput<float>("in_1"),
                            flow::makeInput<float>("in_2")});


            registerCallback({"in_1", "in_2"}, 
                                    [&](flow::DataFlow _data){
                                        auto v1 = _data.get<float>("in_1");
                                        auto v2 = _data.get<float>("in_2"); 
                                             
                                        getPipe("result")->flush(v1-v2);
                                    }
            );
        }


        BlockMultiply::BlockMultiply(){
            createPipe<float>("result");

            createPolicy({  flow::makeInput<float>("in_1"),
                            flow::makeInput<float>("in_2")});


            registerCallback({"in_1", "in_2"}, 
                                    [&](flow::DataFlow _data){
                                        auto v1 = _data.get<float>("in_1");
                                        auto v2 = _data.get<float>("in_2"); 
                                             
                                        getPipe("result")->flush(v1*v2);
                                    }
            );
        }


        BlockDivide::BlockDivide(){
            createPipe<float>("result");

            createPolicy({  flow::makeInput<float>("in_1"),
                            flow::makeInput<float>("in_2")});


            registerCallback({"in_1", "in_2"}, 
                                    [&](flow::DataFlow _data){
                                        auto v1 = _data.get<float>("in_1");
                                        auto v2 = _data.get<float>("in_2"); 
                                             
                                        getPipe("result")->flush(v1/v2);
                                    }
            );
        }
        

        BlockSquareRoot::BlockSquareRoot(){
            createPipe<float>("result");

            createPolicy({  flow::makeInput<float>("in")});


            registerCallback({"in"}, 
                                    [&](flow::DataFlow _data){
                                        auto v1 = _data.get<float>("in");
                                        getPipe("result")->flush(sqrt(v1));
                                    }
            );
        }
        
        BlockPow::BlockPow(){
            createPipe<float>("result");

            createPolicy({  flow::makeInput<float>("base"),
                            flow::makeInput<float>("exp")});


            registerCallback({"base", "exp"}, 
                                    [&](flow::DataFlow _data){
                                        auto v1 = _data.get<float>("base");
                                        auto v2 = _data.get<float>("exp");
                                             
                                        getPipe("result")->flush(float(pow(v1,v2)));
                                    }
            );
        }
    }
}