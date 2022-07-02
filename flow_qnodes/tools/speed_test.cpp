//---------------------------------------------------------------------------------------------------------------------
//  FLOW
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 - Pablo Ramon Soria (a.k.a. Bardo91) 
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

#include <flow/Policy.h>
#include <flow/Outpipe.h>

#include <flow/DataFlow.h>
#include <condition_variable> 

#include <fstream>

#include<fastcom/fastcom.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

using namespace flow;


FLOW_TYPE_REGISTER(int, int)

const int N_TESTS = 10000;

void speedTestFlow();
void speedTestROS();
void speedTestFastcom();

int main(int _argc, char** _argv){
    ros::init(_argc, _argv, "speed_test");
    // Speed test flow
    speedTestFlow();
    speedTestFastcom();
    speedTestROS();
    
}

void speedTestFastcom(){
    std::cout << "Starting fastcom test" <<std::endl;
    fastcom::Publisher<int> publisher(100024);
    fastcom::Subscriber<int> subscriber("127.0.0.1", 100024);
    while(!subscriber.isConnected()){
        std::this_thread::sleep_for(std::chrono::milliseconds(30));   
    }
 
    std::vector<double> times;
    std::mutex locker;
    auto t0 = std::chrono::high_resolution_clock::now();
    auto t1 = std::chrono::high_resolution_clock::now();
    subscriber.attachCallback([&](int &_data){
        t1 = std::chrono::high_resolution_clock::now();
        double millis = std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count()*1e-9;
        times.push_back(millis);
        locker.unlock();
    });

    for(unsigned i = 0; i < N_TESTS; i++){
        locker.lock();
        t0 = std::chrono::high_resolution_clock::now();
        publisher.publish(1);

        if(i != 0) for(unsigned j = 0; j < std::to_string(i-1).size(); j++) std::cout << '\b';
        std::cout << i;
    }
    std::cout << std::endl;
    
    // Save
    std::ofstream file("times_fastcom_int.txt");
    for(auto &t: times){
        file << t << "\n";
    }
}

void speedTestROS(){
    ros::AsyncSpinner spinner(4);
    spinner.start();
    std::cout << "Starting ROS test" <<std::endl;
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<std_msgs::Int32>("/test_speed/int", 1);

    std::vector<double> times;
    std::mutex locker;
    auto t0 = std::chrono::high_resolution_clock::now();
    auto t1 = std::chrono::high_resolution_clock::now();
    ros::Subscriber subscriber = nh.subscribe<std_msgs::Int32>("/test_speed/int", 1, [&](const std_msgs::Int32::ConstPtr &_msgs){
        t1 = std::chrono::high_resolution_clock::now();
        double millis = std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count()*1e-9;
        times.push_back(millis);
        locker.unlock();
    });


    for(unsigned i = 0; i < N_TESTS && ros::ok(); i++){
        locker.lock();
        t0 = std::chrono::high_resolution_clock::now();
        std_msgs::Int32 msg; msg.data = 1;
        publisher.publish(msg);

        if(i != 0) for(unsigned j = 0; j < std::to_string(i-1).size(); j++) std::cout << '\b';
        std::cout << i;
    }
    std::cout << std::endl;


    // Save
    std::ofstream file("times_ros_int.txt");
    for(auto &t: times){
        file << t << "\n";
    }

}

void speedTestFlow(){
    std::cout << "Starting flow test" <<std::endl;
    Outpipe op("counter", "int");
    Policy pol({{{"counter", "int"}}});

    std::vector<double> times;
    std::mutex locker;
    auto t0 = std::chrono::high_resolution_clock::now();
    auto t1 = std::chrono::high_resolution_clock::now();
    pol.registerCallback({"counter"}, [&](DataFlow _f){
        t1 = std::chrono::high_resolution_clock::now();
        double millis = std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count()*1e-9;
        times.push_back(millis);
        locker.unlock();
    });

    op.registerPolicy(&pol, "counter");

    // FLUSH
    for(unsigned i = 0; i < N_TESTS; i++){
        locker.lock();
        t0 = std::chrono::high_resolution_clock::now();
        op.flush(1);
        
        if(i != 0) for(unsigned j = 0; j < std::to_string(i-1).size(); j++) std::cout << '\b';
        std::cout << i;
    }
    std::cout << std::endl;

    // Save
    std::ofstream file("times_flow_int.txt");
    for(auto &t: times){
        file << t << "\n";
    }
}