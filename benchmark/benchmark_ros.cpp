//-----------------------------------------------------------------------------
//  Arduino MICO plugin
//-----------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//-----------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to
//  deal in the Software without restriction, including without limitation the
//  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
//  sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//  IN THE SOFTWARE.
//-----------------------------------------------------------------------------

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <map>
#include <mutex>
#include <thread>

void singleIntStream(int nIters);
void multipleIntStream(int nStreams, int nIters);
void singleCvStream();

int main(int _argc, char **_argv) {
  ros::init(_argc, _argv, "benchmark_ros");

  ros::AsyncSpinner spin(4);
  spin.start();
  {
    std::vector<double> sIters = {1e1, 1e2, 1e3, 1e4, 1e5};
    for (auto i : sIters)
      singleIntStream(i);
  }

  {
    std::vector<double> nStreamers = {5, 10, 20, 50};
    for (auto s : nStreamers) {
      multipleIntStream(s, 1000);
    }
  }
}

void singleIntStream(int nIters) {
  ros::NodeHandle nh;

  std::ofstream file("bm_ros_singleInt_" + std::to_string(nIters) + ".txt");

  std::vector<double> times(nIters);
  std::vector<std::chrono::system_clock::time_point> t0s(nIters);

  boost::function<void(const std_msgs::Int32)> cb =
      [&](const std_msgs::Int32 &_data) {
        auto t1 = std::chrono::system_clock::now();
        int i = _data.data;
        times[i] =
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0s[i])
                .count();
      };

  auto sub1 = nh.subscribe<std_msgs::Int32>("topic", 0, cb);

  auto pub1 = nh.advertise<std_msgs::Int32>("topic", 0);

  for (int i = 0; i < nIters; i++) {
    std_msgs::Int32 data;
    data.data = i;
    t0s[i] = std::chrono::system_clock::now();
    pub1.publish(data);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  double acc = 0;
  std::for_each(times.begin(), times.end(), [&](double _a) {
    acc += _a / nIters;
    file << _a << ",";
  });
  file << std::endl;
  std::cout << "Avg time int flush: " << acc << "ns" << std::endl;

  file.flush();
  file.close();
}

void multipleIntStream(int nStreams, int nIters) {
  ros::NodeHandle nh;

  std::vector<ros::Subscriber> policies(nStreams);
  std::vector<ros::Publisher> pipes(nStreams);
  std::vector<std::thread> callers(nStreams);

  std::map<int, std::vector<double>> times;
  std::map<int, std::vector<std::chrono::system_clock::time_point>> t0s;

  boost::function<void(const std_msgs::Int32, int)> cb =
      [&](const std_msgs::Int32 &_data, int _id) {
        auto t1 = std::chrono::system_clock::now();
        int i = _data.data;
        times[_id][i] = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            t1 - t0s[_id][i])
                            .count();
      };

  std::mutex m;
  std::condition_variable cv;

  for (unsigned i = 0; i < nStreams; i++) {
    times[i] = std::vector<double>(nIters);
    t0s[i] = std::vector<std::chrono::system_clock::time_point>(nIters);

    boost::function<void(const std_msgs::Int32)> cbf =
        boost::bind(cb, boost::placeholders::_1, i);
    policies[i] =
        nh.subscribe<std_msgs::Int32>("topic" + std::to_string(i), 0, cbf);
    pipes[i] = nh.advertise<std_msgs::Int32>("topic" + std::to_string(i), 0);

    callers[i] = std::thread(std::bind(
        [&](int id) {
          std::unique_lock<std::mutex> lk(m);
          cv.wait(lk);

          for (int iter = 0; iter < nIters; iter++) {
            t0s[id][iter] = std::chrono::system_clock::now();
            std_msgs::Int32 data;
            data.data = iter;
            pipes[id].publish(data);
          }
        },
        i));
  }

  cv.notify_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::ofstream file("bm_ros_multiInt_" + std::to_string(nStreams) + "_" +
                     std::to_string(nIters) + ".txt");

  long iters = 0;
  double acc = 0;
  for (auto &[id, subTimes] : times) {
    for (auto &t : subTimes) {
      file << t << ",";
      acc += t / nIters * nStreams;
    }
    file << std::endl;
  }
  std::cout << "Benchmark multithread: " << acc << "ns" << std::endl;
  file.close();

  for (auto &caller : callers) {
    caller.join();
  }
}