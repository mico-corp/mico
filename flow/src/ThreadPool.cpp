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

#include <flow/ThreadPool.h>

namespace flow{
    std::shared_ptr<ThreadPool> ThreadPool::instance_ = nullptr;

    void ThreadPool::init(size_t _nThreads){
        if(!instance_)
            instance_ = std::shared_ptr<ThreadPool>(new ThreadPool(_nThreads));
    }

    std::shared_ptr<ThreadPool> ThreadPool::get(){
        if(!instance_)
            init(std::thread::hardware_concurrency());
        return instance_;
    }

    void ThreadPool::emplace(Task _task){
        {
            std::unique_lock<std::mutex> lock(threadLock_);
            tasks_.emplace(std::move(_task));
            waitEvent_.notify_one();
        }
    }

    ThreadPool::~ThreadPool(){
        isRunning_ = false;
        waitEvent_.notify_all();
        for(auto &t:threads_){
            if(t.joinable())
                t.join();
        }
    }

    ThreadPool::ThreadPool(size_t _nThreads){
        nThreads_ = _nThreads;
        threads_.reserve(nThreads_);
        for(unsigned i =0; i < nThreads_; i++){
            threads_.emplace_back([&](){
                while(isRunning_){
                    Task task;
                    {
                        std::unique_lock<std::mutex> lock(threadLock_);
                        waitEvent_.wait(lock, [&](){return !isRunning_ || !tasks_.empty();});
                    
                        task = std::move(tasks_.front());
                        tasks_.pop();
                    }
                    if(task)
                        task();
                }
            });
        }
    }
}