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
#include <flow/ThreadPool.h>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <algorithm>
#include <map>

using namespace flow;

void singleIntStream();
void multipleIntStream();
void singleCvStream();

int main(void){
    singleIntStream();
    multipleIntStream();

}


void singleIntStream(){
    auto pol1 = new Policy( { makeInput<int>("pol1") } );
    auto pipe1 = new Outpipe("pol1", typeid(int).name());

    std::mutex m;
    std::condition_variable cv;
    
    int nIters = 1e5;
    std::vector<double> times(nIters);
    std::vector<std::chrono::system_clock::time_point> t0s(nIters);

    pipe1->registerPolicy(pol1, "pol1");
    pol1->registerCallback({"pol1"},
    [&](DataFlow _data){
        auto t1 = std::chrono::system_clock::now();
        int i = _data.get<int>("pol1");
        times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0s[i]).count();
        // printf("Times %d", i);  
    });

    for(int i = 0 ; i < nIters ; i++){
        t0s[i] = std::chrono::system_clock::now();
        pipe1->flush(i);
    }
    

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    while(ThreadPool::get()->queueSize()){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    double acc = std::accumulate(times.begin(), times.end(), 0);
    double avg = acc / times.size();
    std::cout << "Avg time int flush: "  << avg << "ns" << std::endl;

}


void multipleIntStream(){

    int nStreams = 50;
    int nIters = 1e3;
    std::vector<Policy*> policies(nStreams);
    std::vector<Outpipe*> pipes(nStreams);
    std::vector<std::thread> callers(nStreams);

    std::map<int, std::vector<double>> times;
    std::map<int, std::vector<std::chrono::system_clock::time_point>> t0s;
    
    std::mutex m;
    std::condition_variable cv;
    
    for(unsigned i = 0; i < nStreams;i++){
        times[i] = std::vector<double>(nIters);
        t0s[i] = std::vector<std::chrono::system_clock::time_point>(nIters);

        policies[i] = new Policy( { makeInput<int>("pol1") } );
        pipes[i] = new Outpipe("pol1", typeid(int).name());


        pipes[i]->registerPolicy(policies[i], "pol1");
        policies[i]->registerCallback({"pol1"},
        std::bind([&](DataFlow _data, int id){
            auto t1 = std::chrono::system_clock::now();
            int iter = _data.get<int>("pol1");
            times[id][iter] = std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0s[id][iter]).count();
            // printf("Id %d, iter %d\n", id, iter);
        }, std::placeholders::_1, i));

        callers[i] = std::thread(std::bind(
            [&](int id){
                std::unique_lock<std::mutex> lk(m);
                cv.wait(lk);

                for(int iter = 0 ; iter < nIters ; iter++){
                    t0s[id][iter] = std::chrono::system_clock::now();
                    pipes[id]->flush(iter);
                }
            }, i)
            );
    }


    cv.notify_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    while(ThreadPool::get()->queueSize()){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    long iters = 0;
    double acc = 0;
    for(auto ts:times){
        acc += std::accumulate(ts.second.begin(), ts.second.end(), 0);
        iters += ts.second.size();
    }
    double avg = acc / iters;
    std::cout << "Benchmark multithread: "  << avg << "ns" << std::endl;

}