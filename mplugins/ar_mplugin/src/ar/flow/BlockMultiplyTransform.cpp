//---------------------------------------------------------------------------------------------------------------------
//  AR MICO plugin
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


#include <mico/ar/flow/BlockMultiplyTransform.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

namespace mico{
    namespace ar {
        BlockMultiplyTransform::BlockMultiplyTransform(){
            createPipe<Eigen::Matrix4f>("T1*T2");

            createPolicy({  flow::makeInput<Eigen::Matrix4f>("T1"),
                            flow::makeInput<Eigen::Matrix4f>("T2") });

            registerCallback({"T1", "T2"},
                &BlockMultiplyTransform::policyCallback,
                this
            );
        }

        void BlockMultiplyTransform::policyCallback(Eigen::Matrix4f _t1, Eigen::Matrix4f _t2) {

            if (getPipe("T1*T2")->registrations()) {
                Eigen::Matrix4f mul = _t1 * _t2;
                getPipe("T1*T2")->flush(mul);
            }



        }
    }
}