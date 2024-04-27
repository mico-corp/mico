//-----------------------------------------------------------------------------
//  FLOW
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

#ifndef FLOW_THREADPOOL_H_
#define FLOW_THREADPOOL_H_

#include <condition_variable>
#include <cstdint>
#include <flow/Export.h>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace flow {

/// Base class of flow that handles the creation of multiple threads to optimize
/// their usage over the flow architecture.
/// @ingroup  flow
class ThreadPool {
public: 
  using Task = std::function<void()>;
private:
class ThreadPoolImpl{
  public:
  ThreadPoolImpl(size_t _nThreads);

  static void init();
  static void deinit();
  static ThreadPoolImpl *get();
  FLOW_DECL void emplace(Task _task);

  ~ThreadPoolImpl();
  size_t queueSize() { return tasks_.size(); }
  float loadRatio() { return float(tasks_.size()) / nThreads_; }

private:
  static std::unique_ptr<ThreadPoolImpl> instance_;
  static int numInstances_;

  std::queue<Task> tasks_;
  bool isRunning_ = true;
  size_t nThreads_;
  std::vector<std::thread> threads_;
  std::condition_variable waitEvent_;
  std::mutex threadLock_;
}; // class ThreadPoolImpl

public:
  ThreadPool(){
    notOwnedThreadPool_ = ThreadPoolImpl::get();
  }

  ~ThreadPool(){
    ThreadPoolImpl::deinit();
  }

  FLOW_DECL void emplace(Task _task){
    notOwnedThreadPool_->emplace(_task);
  }

  FLOW_DECL size_t queueSize() { 
    return notOwnedThreadPool_->queueSize();
  }
  FLOW_DECL float loadRatio() {
    return notOwnedThreadPool_->loadRatio();
  }

private:
  ThreadPoolImpl* notOwnedThreadPool_ = nullptr;


}; // class ThreadPool

} // namespace flow

#endif