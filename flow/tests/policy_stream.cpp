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

#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <flow/DataFlow.h>

#include <condition_variable>
#include <gtest/gtest.h>

using namespace flow;

TEST(policy_creation, policy_creation) {
  Policy pol1({makeInput<float>("time")});
  Policy pol2({makeInput<float>("time"), makeInput<int>("counter")});
  EXPECT_THROW(Policy pol3({PolicyInput("", "")}), std::invalid_argument);
  EXPECT_THROW(Policy pol4({PolicyInput("", "int")}), std::invalid_argument);
  EXPECT_THROW(Policy pol5({PolicyInput("time", "")}), std::invalid_argument);
}

TEST(pipe_creation, pipe_creation) {

  EXPECT_THROW(Outpipe op("", ""), std::invalid_argument);

  [[maybe_unused]] Outpipe *op1 = makeOutput<int>("A");
  [[maybe_unused]] Outpipe *op2 = makeOutput<float>("B");
  [[maybe_unused]] Outpipe *op3 = makeOutput<float>("A");
  [[maybe_unused]] Policy pol({makeInput<int>("A"), makeInput<float>("B")});
}

TEST(registration, registration) {
  // Invalid creation

  // Valid creation
  Outpipe *op = makeOutput<int>("o1");
  ASSERT_STREQ("o1", op->tag().c_str());
  ASSERT_EQ(0, op->registrations());

  Policy pol({makeInput<int>("o1")});
  ASSERT_TRUE(op->registerPolicy(&pol, "o1"));
  ASSERT_FALSE(op->registerPolicy(&pol, "o"));
  ASSERT_EQ(1, op->registrations());

  //
  Outpipe *op1 = makeOutput<int>("A");
  Outpipe *op2 = makeOutput<float>("B");
  Outpipe *op3 = makeOutput<float>("A");
  Policy pol1({makeInput<int>("A"), makeInput<float>("B")});

  ASSERT_TRUE(op1->registerPolicy(&pol1, "A"));
  ASSERT_TRUE(op2->registerPolicy(&pol1, "B"));

  ASSERT_FALSE(op2->registerPolicy(&pol1, "C"));
  ASSERT_FALSE(op3->registerPolicy(&pol1, "A"));
}

TEST(transmission_int_1, transmission_int) {
  // Tags of Output and Policy do not need to be the same.
  Outpipe *op = makeOutput<int>("number");

  Policy pol({makeInput<int>("counter")});

  int res = 0;
  pol.registerCallback<int>({"counter"}, [&](int _i1) { res = _i1; });

  op->registerPolicy(&pol, "counter");

  // Good type flush
  op->flush(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_EQ(res, 1);
}

TEST(disconnect, disconnect) {
  Outpipe *op = makeOutput<int>("counter");
  Policy pol({makeInput<int>("counter")});

  int res = 0;
  bool goodCallback =
      pol.registerCallback<int>({"counter"}, [&](int _i1) { res = _i1; });
  ASSERT_TRUE(goodCallback);

  bool badCallback = pol.registerCallback<float>({"float"}, [&](float) {});
  ASSERT_FALSE(badCallback);

  op->registerPolicy(&pol, "counter");

  op->flush(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_EQ(res, 1);

  pol.disconnect("counter");

  op->flush(2);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_EQ(res, 1);
  ASSERT_EQ(0, op->registrations());
}

TEST(transmission_int_2, transmission_int) {
  Outpipe *op = makeOutput<int>("counter");

  Policy pol1({makeInput<int>("counter")});
  Policy pol2({makeInput<int>("counter")});

  int counterCall1 = 0;
  pol1.registerCallback<int>({"counter"}, [&](int) { counterCall1++; });

  // Intentionally block one of the policies.
  // Flow does not pipe triggers, if the policy is busy when a
  // new trigger arrives, the signal is dropped
  std::mutex mtx;
  std::condition_variable cv;
  int counterCall2 = 0;
  pol2.registerCallback<int>({"counter"}, [&](int) {
    std::unique_lock<std::mutex> lck(mtx);
    cv.wait(lck);
    counterCall2++;
  });

  op->registerPolicy(&pol1, "counter");
  op->registerPolicy(&pol2, "counter");

  //
  op->flush(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  op->flush(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  op->flush(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  op->flush(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  cv.notify_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  ASSERT_EQ(4, counterCall1);
  ASSERT_EQ(1,
            counterCall2); // As the task did not end  until mutex is released,
                           // the rest of flushes did not generate new tasks

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

TEST(sync_policy, sync_policy) {
  Outpipe *op1 = makeOutput<int>("counter");
  Outpipe *op2 = makeOutput<float>("clock");

  Policy pol({makeInput<int>("counter"), makeInput<float>("clock")});

  int counterCallInt = 0;
  int counterCallFloat = 0;
  int counterCallSync = 0;
  pol.registerCallback<int>({"counter"}, [&](int) { counterCallInt++; });
  pol.registerCallback<float>({"clock"}, [&](float) { counterCallFloat++; });
  pol.registerCallback<int, float>({"counter", "clock"},
                                   [&](int, float) { counterCallSync++; });

  op1->registerPolicy(&pol, "counter");
  op2->registerPolicy(&pol, "clock");

  // Good type flush
  op1->flush(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  op2->flush(1.123f);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  ASSERT_EQ(1, counterCallInt);
  ASSERT_EQ(1, counterCallFloat);
  ASSERT_EQ(1, counterCallSync);
}

TEST(deep_chain, deep_chain) {
  Outpipe *op = makeOutput<int>("counter");
  Policy pol({makeInput<int>("ticks")});
  op->registerPolicy(&pol, "ticks");

  Outpipe *op2 = makeOutput<int>("accum");
  Policy pol2({makeInput<int>("subs")});
  op2->registerPolicy(&pol2, "subs");

  Outpipe *op3 = makeOutput<int>("josua");
  Policy pol3({makeInput<int>("johny")});
  op3->registerPolicy(&pol3, "johny");

  pol.registerCallback<int>({"ticks"}, [&](int _in) {
    int res = 2 * _in;
    ASSERT_EQ(res, 4);
    op2->flush(res);
  });

  pol2.registerCallback<int>({"subs"}, [&](int _in) {
    int res = 2 * _in;
    ASSERT_EQ(res, 8);
    op3->flush(res);
  });

  bool called = false;
  pol3.registerCallback<int>({"johny"}, [&](int _in) {
    int res = 2 * _in;
    ASSERT_EQ(res, 16);
    called = true;
  });

  op->flush(2);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_TRUE(called);
}

TEST(deep_chain_split, deep_chain_split) {
  //
  // Structure of test
  //                             |--> o2 --> p2 --> o4 --> p4
  //  o0 --> p0 --> o1 --> p1 -->|
  //                             |--> o3 --> p3 --> o5 --> p5
  //
  //  2------4--------------8----------------16------------32------Check here

  std::vector<Outpipe *> pipes;
  std::vector<Policy *> policies;

  for (unsigned i = 0; i < 7; i++) {
    pipes.push_back(makeOutput<int>("counter"));
    policies.push_back(new Policy({makeInput<int>("counter")}));
    pipes.back()->registerPolicy(policies.back(), "counter");
  }

  policies[0]->registerCallback<int>({"counter"}, [&](int _in) {
    int res = 2 * _in;
    pipes[1]->flush(res);
  });

  policies[1]->registerCallback<int>({"counter"}, [&](int _in) {
    int res = 2 * _in;
    pipes[2]->flush(res);
    pipes[3]->flush(res);
  });

  // Branch 1
  policies[2]->registerCallback<int>({"counter"}, [&](int _in) {
    int res = 2 * _in;
    pipes[4]->flush(res);
  });

  int nCounterB1 = 0;
  policies[4]->registerCallback<int>({"counter"}, [&](int _in) {
    int res = 2 * _in;
    ASSERT_EQ(res, 32);
    nCounterB1++;
  });

  // Branch 2
  policies[3]->registerCallback<int>({"counter"}, [&](int _in) {
    int res = 2 * _in;
    pipes[5]->flush(res);
  });

  int nCounterB2 = 0;
  policies[5]->registerCallback<int>({"counter"}, [&](int _in) {
    int res = 2 * _in;
    ASSERT_EQ(res, 32);
    nCounterB2++;
  });

  pipes.front()->flush(2);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ASSERT_EQ(nCounterB1, 1);
  ASSERT_EQ(nCounterB2, 1);
}

TEST(loop_chain_split, loop_chain_split) {
  //
  // Structure of test
  //
  //         v--------------------------|
  //  o0 --> p0 --> o1 --> p1 -|--> o3 -|-> p3
  //                           |
  //                           |--> o4 --> p4

  // Create pipes and connections
  Outpipe *o0 = makeOutput<int>("counter");
  Policy p0({makeInput<int>("counter"), makeInput<std::string>("msg")});

  o0->registerPolicy(&p0, "counter");

  Outpipe *o1 = makeOutput<int>("ticks");
  Policy p1({makeInput<int>("tocks")});
  o1->registerPolicy(&p1, "tocks");

  Outpipe *o3 = makeOutput<std::string>("msg");
  Policy p3({makeInput<std::string>("message")});
  o3->registerPolicy(&p0, "msg");
  o3->registerPolicy(&p3, "message");

  Outpipe *o4 = makeOutput<int>("cnt");
  Policy p4({makeInput<int>("counter")});
  o4->registerPolicy(&p4, "counter");

  // Set callbacks
  bool calledOnce0a = true;
  std::mutex guard0a;
  p0.registerCallback<int>({"counter"}, [&](int) {
    guard0a.lock();
    o1->flush(1);
    ASSERT_TRUE(calledOnce0a);
    calledOnce0a = false;
    guard0a.unlock();
  });

  bool calledOnce0b = true;
  std::mutex guard0b;
  p0.registerCallback<std::string>({"msg"}, [&](std::string) {
    guard0b.lock();
    ASSERT_TRUE(calledOnce0b);
    calledOnce0b = false;
    guard0b.unlock();
  });

  p1.registerCallback<int>({"tocks"}, [&](int) {
    o3->flush("pepe");
    o4->flush(1);
  });

  bool calledOnce4 = true;
  std::mutex guard4;
  p4.registerCallback<int>({"counter"}, [&](int) {
    guard4.lock();
    ASSERT_TRUE(calledOnce4);
    calledOnce4 = false;
    guard4.unlock();
  });

  o0->flush(1);
  while (calledOnce4) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

TEST(concurrency_attack_test, concurrency_attack_test) {
  Outpipe *op = makeOutput<int>("counter");

  Policy pol({makeInput<int>("cnt")});

  int counterCall1 = 0;
  bool idle = true;
  pol.registerCallback<int>({"cnt"}, [&](int) {
    if (idle) {
      idle = false;
      counterCall1++;
      std::this_thread::sleep_for(std::chrono::milliseconds(
          100)); // To ensure that no one else is comming in
      // std::cout << "jeje, I am the one which got into " <<
      // std::this_thread::get_id() << std::endl;
      idle = true;
    } else {
      // std::cout << "Shit I failed" << std::this_thread::get_id() <<
      // std::endl;
    }
  });

  op->registerPolicy(&pol, "cnt");

  std::mutex mtx;
  std::condition_variable cv;
  bool ready = false;

  auto print_id = [&](int) {
    std::unique_lock<std::mutex> lck(mtx);
    while (!ready)
      cv.wait(lck);
    op->flush(1);
  };

  const int nThreads = 10;
  std::thread vThreads[nThreads];
  for (int i = 0; i < nThreads; ++i)
    vThreads[i] = std::thread(print_id, i);

  {
    std::unique_lock<std::mutex> lck(mtx);
    ready = true;
    cv.notify_all();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  for (auto &th : vThreads)
    th.join();

  ASSERT_EQ(1, counterCall1);
}

TEST(multiple_register_policies_test, multiple_register_policies_test) {

  // This does not work yet 666
  //
  // o1  ---------------------  i1
  //       |
  //       |------------------  i2
  //
  // o2  ---------------------  i3
  //

  // Policy p({ makeInput<int>("i1") , makeInput<int>("i2"),
  // makeInput<int>("i3") });

  // Outpipe* o1 = makeOutput<int>("o1");
  // o1->registerPolicy(&p, "i1");
  // o1->registerPolicy(&p, "i2");

  // Outpipe* o2 = makeOutput<int>("o2");
}

TEST(clear_data_test, clear_data_test) {

  // // Prepare a simple policy with 2 inputs
  // Policy p({makeInput<int>("i1"), makeInput<int>("i2")});

  // // Prepare two outputs to feed the policy
  // Outpipe *o1 = makeOutput<int>("o1");
  // o1->registerPolicy(&p, "i1");

  // Outpipe *o2 = makeOutput<int>("o2");
  // o2->registerPolicy(&p, "i2");

  // // Register a callback for one tag
  // volatile int i1 = 0;
  // p.registerCallback<int>({"i1"}, [&](int _in) { i1 = _in; });

  // // Register a callback for the other tag
  // volatile int i2 = 0;
  // p.registerCallback<int>({"i2"}, [&](int _in) { i2 = _in; });

  // // Register a callback for both, this callback do not clear data flows.
  // volatile int iCombi1 = 0;
  // p.registerCallback<int, int>({"i1", "i2"},
  //                              [&](int _i1, int _i2) { iCombi1 = _i1 + _i2; });

  // // At the moment just one input has data, so the combination is still 0
  // o1->flush(1);
  // while (i1==0) { }
  // ASSERT_EQ(i1, 1);
  // ASSERT_EQ(iCombi1, 0);

  // // Now both inputs are fed, the combined is called
  // o2->flush(3);
  // while (i2==0 || iCombi1==0) { }
  
  // ASSERT_EQ(i2, 3);
  // ASSERT_EQ(iCombi1, 4);

  // // Register a new callback for both, but in this case, the callback clears the
  // // data used
  // int iCombi2 = 0;
  // p.registerCallback<int, int>({"i1", "i2"},
  //                              [&](int _i1, int _i2) { iCombi2 = _i1 + _i2; });

  // // The first combi already has data, so it changes
  // o1->flush(5);
  // while (i1 != 5 || iCombi1 != 5) { }
  // ASSERT_EQ(i1, 5);
  // ASSERT_EQ(iCombi1, 8);
  // ASSERT_EQ(iCombi2, 0);

  // // Now both data input are fed, second combi is called, but flags are cleared
  // o1->flush(5);
  // o2->flush(0);
  // while (i2 != 0 || iCombi1 != 5 || iCombi2 != 5) { }
  // ASSERT_EQ(i2, 0);
  // ASSERT_EQ(iCombi1, 5);
  // ASSERT_EQ(iCombi2, 5);

  // // As expected, the second combi value did not change because i2 input is not
  // // fed again
  // o1->flush(10);
  // while (i1 == 5 || iCombi1 == 5) { }
  // ASSERT_EQ(i1, 10);
  // ASSERT_EQ(iCombi1, 10);
  // ASSERT_EQ(iCombi2, 5);
}

int main(int _argc, char **_argv) {
  testing::InitGoogleTest(&_argc, _argv);
  return RUN_ALL_TESTS();
}
