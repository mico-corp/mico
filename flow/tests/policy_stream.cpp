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

#include <gtest/gtest.h>
#include <condition_variable> 

using namespace flow;

TEST(policy_creation, policy_creation)  {
    Policy pol1({{"time", "float"}});
    Policy pol2({{"time", "float"}, {"counter", "int"}});
    EXPECT_THROW(Policy pol3({{"", ""}}), std::invalid_argument);
    EXPECT_THROW(Policy pol4({{"", "int"}}), std::invalid_argument);
    EXPECT_THROW(Policy pol5({{"time", ""}}), std::invalid_argument);
}

TEST(pipe_creation, pipe_creation)  {

    EXPECT_THROW(Outpipe op("", ""), std::invalid_argument);
    
    Outpipe op1("A","int");
    Outpipe op2("B","float");
    Outpipe op3("A","float");
    Policy pol({{"A","int"},{"B","float"}});

}

TEST(registration, registration)  {
    // Invalid creation

    // Valid creation
    Outpipe op("o1", "int");
    ASSERT_STREQ("o1", op.tag().c_str());
    ASSERT_EQ(0, op.registrations());

    Policy pol({{"o1", "int"}});
    ASSERT_TRUE(op.registerPolicy(&pol, "o1"));
    ASSERT_FALSE(op.registerPolicy(&pol, "o"));
    ASSERT_EQ(1, op.registrations());

    //
    Outpipe op1("A","int");
    Outpipe op2("B","float");
    Outpipe op3("A","float");
    Policy pol1({{"A","int"},{"B","float"}});

    ASSERT_TRUE(op1.registerPolicy(&pol1, "A"));
    ASSERT_TRUE(op2.registerPolicy(&pol1, "B"));

    ASSERT_FALSE(op2.registerPolicy(&pol1, "C"));
    ASSERT_FALSE(op3.registerPolicy(&pol1, "A"));

}

TEST(transmission_int_1, transmission_int)  {
    Outpipe op("counter", "int");

    Policy pol({{"counter", "int"}});

    int res = 0;
    pol.registerCallback({"counter"}, [&](DataFlow _f){
        res = _f.get<int>("counter");
    });

    op.registerPolicy(&pol, "counter");

    // Good type flush
    op.flush(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_EQ(res, 1);

    // Bad type flush
    EXPECT_THROW(
        op.flush(5.312f);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    , std::bad_any_cast);
}


TEST(disconnect, disconnect)  {
    Outpipe op("counter","int");
    Policy pol({{"counter","int"}});

    int res = 0;
    bool goodCallback = pol.registerCallback({"counter"}, [&](DataFlow _f){
        res = _f.get<int>("counter");
    });
    ASSERT_TRUE(goodCallback);

    bool badCallback = pol.registerCallback({"float"}, [&](DataFlow _f){ });
    ASSERT_FALSE(badCallback);
    
    op.registerPolicy(&pol, "counter");
    
    op.flush(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(res, 1);

    pol.disconnect("counter");

    op.flush(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(res, 1);
    ASSERT_EQ(0, op.registrations());
}

TEST(transmission_int_2, transmission_int)  {
    Outpipe op("counter","int");

    Policy pol1({{"counter","int"}});
    Policy pol2({{"counter","int"}});

    int counterCall1 = 0;
    std::mutex guardCall1;
    pol1.registerCallback({"counter"}, [&](DataFlow _f){
        guardCall1.lock();
        counterCall1++;
        guardCall1.unlock();    
    });
    
    int counterCall2 = 0;
    std::mutex guardCall2;
    pol2.registerCallback({"counter"}, [&](DataFlow _f){
        guardCall2.lock();
        counterCall2++;
        guardCall2.unlock();
    });

    op.registerPolicy(&pol1, "counter");
    op.registerPolicy(&pol2, "counter");

    // Good type flush
    op.flush(1);
    op.flush(1);
    op.flush(1);
    op.flush(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(4, counterCall1);
    ASSERT_EQ(4, counterCall2);
}


TEST(sync_policy, sync_policy)  {
    Outpipe op1("counter", "int");
    Outpipe op2("clock", "float");

    Policy pol({{"counter", "int"}, {"clock", "float"}});

    int counterCallInt = 0;
    int counterCallFloat = 0;
    int counterCallSync = 0;
    std::mutex guardCall1;
    pol.registerCallback({"counter"}, [&](DataFlow _f){
        counterCallInt++;    
    });
    pol.registerCallback({"clock"}, [&](DataFlow _f){
        counterCallFloat++;    
    });
    pol.registerCallback({"counter", "clock"}, [&](DataFlow _f){
        counterCallSync++;    
    });
    
    op1.registerPolicy(&pol,"counter");
    op2.registerPolicy(&pol,"clock");

    // Good type flush
    op1.flush(1);
    op2.flush(1.123f);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(1, counterCallInt);
    ASSERT_EQ(1, counterCallFloat);
    ASSERT_EQ(1, counterCallSync);
}



TEST(deep_chain, deep_chain)  {
    Outpipe op("counter", "int");
    Policy pol({{"ticks", "int"}});
    op.registerPolicy(&pol, "ticks");

    Outpipe op2("accum", "int");
    Policy pol2({{"subs", "int"}});
    op2.registerPolicy(&pol2, "subs");

    Outpipe op3("josua", "int");
    Policy pol3({{"johny", "int"}});
    op3.registerPolicy(&pol3, "johny");


    pol.registerCallback({"ticks"}, [&](DataFlow _f){
        int res = 2 * _f.get<int>("ticks");
        ASSERT_EQ(res, 4);
        op2.flush(res);
    });

    pol2.registerCallback({"subs"}, [&](DataFlow _f){
        int res = 2 * _f.get<int>("subs");
        ASSERT_EQ(res, 8);
        op3.flush(res);
    });

    bool called = false;
    pol3.registerCallback({"johny"}, [&](DataFlow _f){
        int res = 2 * _f.get<int>("johny");
        ASSERT_EQ(res, 16);
        called = true;  
    });
    
    op.flush(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_TRUE(called);
}


TEST(deep_chain_split, deep_chain_split)  {
    //
    // Structure of test
    //                             |--> o2 --> p2 --> o4 --> p4
    //  o0 --> p0 --> o1 --> p1 -->|
    //                             |--> o3 --> p3 --> o5 --> p5
    //
    //  2------4--------------8----------------16------------32------Check here

    std::vector<Outpipe*> pipes;
    std::vector<Policy*> policies;

    for(unsigned i = 0; i < 7; i++){
        pipes.push_back(new Outpipe("counter", "int"));
        policies.push_back(new Policy({{"counter", "int"}}));
        pipes.back()->registerPolicy(policies.back(), "counter");
    }


    policies[0]->registerCallback({"counter"}, [&](DataFlow _f){
        int res = 2 * _f.get<int>("counter");
        pipes[1]->flush(res);
    });

    policies[1]->registerCallback({"counter"}, [&](DataFlow _f){
        int res = 2 * _f.get<int>("counter");
        pipes[2]->flush(res);
        pipes[3]->flush(res);
    });

    // Branch 1
    policies[2]->registerCallback({"counter"}, [&](DataFlow _f){
        int res = 2 * _f.get<int>("counter");
        pipes[4]->flush(res);
    });

    int nCounterB1 = 0;
    std::mutex lockerB1;
    policies[4]->registerCallback({"counter"}, [&](DataFlow _f){
        lockerB1.lock();
        int res = 2 * _f.get<int>("counter");
        ASSERT_EQ(res, 32);
        nCounterB1++;
        lockerB1.unlock();
    });

    // Branch 2
    policies[3]->registerCallback({"counter"}, [&](DataFlow _f){
        int res = 2 * _f.get<int>("counter");
        pipes[5]->flush(res);
    });

    int nCounterB2 = 0;
    std::mutex lockerB2;
    policies[5]->registerCallback({"counter"}, [&](DataFlow _f){
        lockerB2.lock();
        int res = 2 * _f.get<int>("counter");
        ASSERT_EQ(res, 32);
        nCounterB2++;
        lockerB2.unlock();
    });

    
    pipes.front()->flush(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ASSERT_EQ(nCounterB1, 1);
    ASSERT_EQ(nCounterB2, 1);
}

TEST(loop_chain_split, loop_chain_split)  {
    //
    // Structure of test
    //                           
    //         v--------------------------| 
    //  o0 --> p0 --> o1 --> p1 -|--> o3 -|-> p3
    //                           |
    //                           |--> o4 --> p4

    // Create pipes and connections
    Outpipe o0("counter","int");
    Policy p0({{"counter","int"}, {"msg","string"}});
    o0.registerPolicy(&p0, "counter");

    Outpipe o1("ticks","int");
    Policy p1({{"tocks","int"}});
    o1.registerPolicy(&p1, "tocks");

    Outpipe o3("msg","string");
    Policy p3({{"message","string"}});
    o3.registerPolicy(&p0, "msg");
    o3.registerPolicy(&p3, "message");

    Outpipe o4("cnt", "int");
    Policy p4({{"counter", "int"}});
    o4.registerPolicy(&p4, "counter");

    // Set callbacks
    bool calledOnce0a = true;
    std::mutex guard0a;
    p0.registerCallback({"counter"}, [&](DataFlow _f){
        guard0a.lock();
        o1.flush(1);
        ASSERT_TRUE(calledOnce0a);
        calledOnce0a = false;
        guard0a.unlock();
    });

    bool calledOnce0b = true;
    std::mutex guard0b;
    p0.registerCallback({"msg"}, [&](DataFlow _f){
        guard0b.lock();
        ASSERT_TRUE(calledOnce0b);
        calledOnce0b = false;
        guard0b.unlock();
    });

    p1.registerCallback({"tocks"}, [&](DataFlow _f){
        o3.flush("pepe");
        o4.flush(1);
    });

    bool calledOnce4 = true;
    std::mutex guard4;
    p4.registerCallback({"counter"}, [&](DataFlow _f){
        guard4.lock();
        ASSERT_TRUE(calledOnce4);
        calledOnce4 = false;
        guard4.unlock();
    });

    o0.flush(1);
    while(calledOnce4){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}



TEST(concurrency_attack_test, concurrency_attack_test)  {
    Outpipe op("counter","int");

    Policy pol({{"cnt", "int"}});

    int counterCall1 = 0;
    bool idle = true;
    pol.registerCallback({"cnt"}, [&](DataFlow _f){
        if(idle){
            idle=false;
            counterCall1++;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));    // To ensure that no one else is comming in
            // std::cout << "jeje, I am the one which got into " << std::this_thread::get_id() << std::endl;
            idle=true;
        }else{
            // std::cout << "Shit I failed" << std::this_thread::get_id() << std::endl;
        }
    });

    op.registerPolicy(&pol, "cnt");    

    std::mutex mtx;
    std::condition_variable cv;
    bool ready = false;

    auto print_id = [&](int id) {
        std::unique_lock<std::mutex> lck(mtx);
        while (!ready) cv.wait(lck);
        op.flush(1);
    };

    const int nThreads = 10;
    std::thread vThreads[nThreads];
    for (int i=0; i<nThreads; ++i)
        vThreads[i] = std::thread(print_id,i);

    {
        std::unique_lock<std::mutex> lck(mtx);
        ready = true;
        cv.notify_all();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (auto& th : vThreads) th.join();

    ASSERT_EQ(1, counterCall1);
}


 
int main(int _argc, char **_argv)  {
    testing::InitGoogleTest(&_argc, _argv);
    return RUN_ALL_TESTS();
}