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


#include<ros/ros.h>
#include <std_msgs/Int32.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <algorithm>
#include <thread>
#include <fstream>
#include <map>

void singleIntStream(int nIters);
void multipleIntStream();
void singleCvStream();

int main(int _argc, char** _argv){
    ros::init( _argc, _argv, "benchmark_ros");

    ros::AsyncSpinner spin(4);
    spin.start();
    {
        std::vector<double> sIters = {1e1,1e2,1e3,1e4,1e5};
        for(auto i: sIters) singleIntStream(i);
    }

    ros::waitForShutdown();

}


void singleIntStream(int nIters){
    ros::NodeHandle nh;

    std::ofstream file("bm_ros_singleInt_"+std::to_string(nIters)+".txt");

    std::vector<double> times(nIters);
    std::vector<std::chrono::system_clock::time_point> t0s(nIters);

    boost::function<void(const std_msgs::Int32)> cb = [&](const std_msgs::Int32 &_data){
        auto t1 = std::chrono::system_clock::now();
        int i = _data.data;
        times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0s[i]).count();
    };

    auto sub1  =  nh.subscribe<std_msgs::Int32>("topic", 0, cb);

    auto pub1 = nh.advertise<std_msgs::Int32>("topic", 0);


    for(int i = 0 ; i < nIters ; i++){
        std_msgs::Int32 data;
        data.data = i;
        t0s[i] = std::chrono::system_clock::now();
        pub1.publish(data);
    }
    

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

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


// void multipleIntStream(){

//     int nStreams = 50;
//     int nIters = 1e3;
//     std::vector<Policy*> policies(nStreams);
//     std::vector<Outpipe*> pipes(nStreams);
//     std::vector<std::thread> callers(nStreams);

//     std::map<int, std::vector<double>> times;
//     std::map<int, std::vector<std::chrono::system_clock::time_point>> t0s;
    
//     std::mutex m;
//     std::condition_variable cv;
    
//     for(unsigned i = 0; i < nStreams;i++){
//         times[i] = std::vector<double>(nIters);
//         t0s[i] = std::vector<std::chrono::system_clock::time_point>(nIters);

//         policies[i] = new Policy( { makeInput<int>("pol1") } );
//         pipes[i] = new Outpipe("pol1", typeid(int).name());


//         pipes[i]->registerPolicy(policies[i], "pol1");
//         policies[i]->registerCallback({"pol1"},
//         std::bind([&](DataFlow _data, int id){
//             auto t1 = std::chrono::system_clock::now();
//             int iter = _data.get<int>("pol1");
//             times[id][iter] = std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0s[id][iter]).count();
//             // printf("Id %d, iter %d\n", id, iter);
//         }, std::placeholders::_1, i));

//         callers[i] = std::thread(std::bind(
//             [&](int id){
//                 std::unique_lock<std::mutex> lk(m);
//                 cv.wait(lk);

//                 for(int iter = 0 ; iter < nIters ; iter++){
//                     t0s[id][iter] = std::chrono::system_clock::now();
//                     pipes[id]->flush(iter);
//                 }
//             }, i)
//             );
//     }


//     cv.notify_all();
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//     while(ThreadPool::get()->queueSize()){
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }

//     long iters = 0;
//     double acc = 0;
//     for(auto ts:times){
//         acc += std::accumulate(ts.second.begin(), ts.second.end(), 0);
//         iters += ts.second.size();
//     }
//     double avg = acc / iters;
//     std::cout << "Benchmark multithread: "  << avg << "ns" << std::endl;

// }