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


#include <mico/audio/flow/BlockDFT.h>
#include <flow/Outpipe.h>
#include <algorithm>
#include <fftw3.h>

namespace mico{
    namespace audio {
        BlockDFT::BlockDFT() {

            createPolicy({ flow::makeInput<std::vector<float>>("signal")});
            registerCallback({ "signal" }, &BlockDFT::computeDFT, this);
            createPipe<std::vector<float>>("dft");
        }


        void BlockDFT::computeDFT(std::vector<float> _signal) {
            const int numPoints = _signal.size();
            if (!numPoints) return;

            if (getPipe("dft")->registrations()) {
                fftw_complex* result = new fftw_complex[size_t(numPoints / 2) + 1];
                std::vector<double> signal(_signal.begin(), _signal.end());

                fftw_plan plan = fftw_plan_dft_r2c_1d(numPoints, signal.data(), result, NULL);
                fftw_execute(plan);


                std::vector<float> output;
                output.resize(numPoints / 2);
                for (int i = 0; i < numPoints / 2; i++) {
                    output[i] = 2.0 / (double)(numPoints)*fabs(result[i][0]);
                }
                delete[] result;
                getPipe("dft")->flush(output);
                
                fftw_destroy_plan(plan);
            }
        }



    }
}
