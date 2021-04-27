//---------------------------------------------------------------------------------------------------------------------
//  FLOW
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

#ifndef FLOW_THREADPOOL_H_
#define FLOW_THREADPOOL_H_

#include <condition_variable>
#include <cstdint>
#include <queue>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace flow{
    
    /// Base class of flow that handles the creation of multiple threads to optimize their usage
    /// over the flow architecture.
    /// @ingroup  flow
    class ThreadPool{
    public:
        using Task = std::function<void()>;

        static void init(size_t _nThreads);
        static std::shared_ptr<ThreadPool> get();
    
        void emplace(Task _task);

        ~ThreadPool();
      
        /// Get size of task queue
        unsigned queueSize() {
            return tasks_.size();
        }
        
    private:
        ThreadPool(size_t _nThreads);
    
    private:
        static std::shared_ptr<ThreadPool> instance_;
        
        std::queue<Task> tasks_;
        bool isRunning_=true;
        int nThreads_;
        std::vector<std::thread> threads_;
        std::condition_variable waitEvent_;
        std::mutex  threadLock_;
    };
}

#endif