//-----------------------------------------------------------------------------
//  FLOW
//-----------------------------------------------------------------------------
//  Copyright 2020 - Pablo Ramon Soria (a.k.a. Bardo91)
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

#include <flow/ThreadPool.h>

#include <gtest/gtest.h>

using namespace flow;

TEST(thread_pool_test, basic_life_cycle) {
  ThreadPool tp;
  tp.loadRatio();
  EXPECT_TRUE(tp.numInstances() == 1);
  EXPECT_TRUE(tp.loadRatio() == 0);
  EXPECT_TRUE(tp.queueSize() == 0);
}

TEST(thread_pool_test, emplace_and_block) {
  ThreadPool tp;

  std::mutex mutex;
  std::atomic<bool> condition = false;
  mutex.lock();

  tp.emplace([&]() {
    std::lock_guard<std::mutex> lock(mutex);
    condition = true;
  });
  EXPECT_TRUE(tp.queueSize() == 1);

  // Release mutex
  mutex.unlock();
  while (!condition) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(tp.queueSize() == 0);
}

int main(int _argc, char **_argv) {
  testing::InitGoogleTest(&_argc, _argv);
  return RUN_ALL_TESTS();
}
