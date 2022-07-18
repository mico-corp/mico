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

#include <cstdlib>
#include <string>
#include <iostream>

namespace flow{
    ThreadPool *ThreadPool::instance_ = nullptr;

    void ThreadPool::init(){
        size_t nThreads = std::thread::hardware_concurrency();
        #if defined(_WIN32)
            char* nThreadsVar = nullptr;
            size_t sz = 0;
            _dupenv_s(&nThreadsVar, &sz, "FLOW_N_THREADS_POOL");
        #elif defined (__linux__)
            const char* nThreadsVar = std::getenv("FLOW_N_THREADS_POOL");
        #endif
        if (nThreadsVar != nullptr) {
            try {
                nThreads = std::stoi(nThreadsVar);
            }
            catch (const std::invalid_argument& _e) {
                std::cout << _e.what() << std::endl;
            }
            catch (const std::out_of_range& _e) {
                std::cout << _e.what() << std::endl;
            }
        }
        if(!instance_)
            instance_ = new ThreadPool(nThreads);

        delete[] nThreadsVar;
    }

    ThreadPool *ThreadPool::get(){
        if(!instance_)
            init();
        return instance_;
    }

    void ThreadPool::emplace(Task _task){
        std::unique_lock<std::mutex> lock(threadLock_);
        tasks_.emplace(std::move(_task));
        waitEvent_.notify_one();
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
        for(size_t i =0; i < nThreads_; i++){
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
