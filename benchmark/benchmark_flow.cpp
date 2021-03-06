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
#include <fstream>
#include <vector>

using namespace flow;

void singleIntStream(int nIters);
void multipleIntStream(int nStreamer, int nIters);
void singleCvStream();

int main(void){

    ThreadPool::init(4);

    {
        std::vector<double> sIters = {1e1,1e2,1e3,1e4,1e5};
        for(auto i: sIters) singleIntStream(i);
    }

    {
        std::vector<double> nStreamers = {5, 10, 20, 50};
        for(auto s:nStreamers){
            multipleIntStream(s, 1000);
        }

    }

}


void singleIntStream(int nIters){
    auto pol1 = new Policy( { makeInput<int>("pol1") } );
    auto pipe1 = new Outpipe("pol1", typeid(int).name());


    std::ofstream file("bm_flow_singleInt_"+std::to_string(nIters)+".txt");

    std::mutex m;
    std::condition_variable cv;
    
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

    double acc = 0;
    std::for_each(times.begin(), times.end(), [&](double _a) { 
        acc += _a/nIters; 
        file << _a << ",";
    });
    file << std::endl;
    std::cout << "Avg time int flush: "  << acc << "ns" << std::endl;

    file.flush();
    file.close();
    
}


void multipleIntStream(int nStreams, int nIters){
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


    std::ofstream file("bm_flow_multiInt_"+std::to_string(nStreams)+"_"+std::to_string(nIters)+".txt");


    long iters = 0;
    double acc = 0;
    for(auto &[id, subTimes]:times){
        for(auto &t: subTimes){
            file << t << ",";
            acc += t / nIters*nStreams;
        }
        file << std::endl;
    }
    std::cout << "Benchmark multithread: "  << acc << "ns" << std::endl;
    file.close();


    for(auto &caller:callers){
        caller.join();
    }
}