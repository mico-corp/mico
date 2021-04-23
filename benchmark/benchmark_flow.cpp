//---------------------------------------------------------------------------------------------------------------------
//  Arduino MICO plugin
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


#include <flow/flow.h>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <algorithm>

using namespace flow;

void singleIntStream();

int main(void){
    singleIntStream();

}


void singleIntStream(){
    auto pol1 = new Policy( { makeInput<int>("pol1") } );
    auto pipe1 = new Outpipe("pol1", typeid(int).name());

    std::mutex m;
    std::condition_variable cv;

    pipe1->registerPolicy(pol1, "pol1");
    
    std::chrono::system_clock::time_point t1;
    pol1->registerCallback({"pol1"},
    [&](DataFlow _data){
        t1 = std::chrono::system_clock::now();
        cv.notify_all();
    });

    int nIters = 1e6;
    std::vector<double> times(nIters);
    for(unsigned i = 0 ; i < nIters ; i++){
        auto t0 = std::chrono::system_clock::now();
        pipe1->flush(1);
        
        std::unique_lock<std::mutex> lk(m);
        cv.wait(lk);
        
        times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count();
    }

    double acc = std::accumulate(times.begin(), times.end(), 0);
    double avg = acc / times.size();
    std::cout << "Avg time int flush: "  << avg << std::endl;

}