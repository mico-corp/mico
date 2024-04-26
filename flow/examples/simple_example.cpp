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

#include <flow/DataFlow.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>
#include <iostream>

using namespace flow;

void testDataFlowCreation();
void testPolicyCreation();

int main(int _argc, char **_argv) {

  testDataFlowCreation();
  testPolicyCreation();
}

void testDataFlowCreation() {

  std::map<std::string, std::string> tags = {{"i1", "int"}, {"i2", "int"}};

  std::function<void(int, int)> cb = [](int _i1, int _i2) {
    std::cout << "I am " << _i1 + _i2 << std::endl;
  };

  DataFlow *df = DataFlow::create(tags, cb);

  df->update("i1", 1);
  df->update("i2", 1);

  std::this_thread::sleep_for(std::chrono::seconds(1));
}

void testPolicyCreation() {
  Policy pol({makeInput<int>("i1"), makeInput<std::string>("i2"),
              makeInput<float>("i3")});

  std::function<void(int)> cb1 = [](int _i1) {
    std::cout << "I am cb1 and just have one argument which value is " << _i1
              << std::endl;
  };

  pol.registerCallback({"i1"}, cb1);

  std::function<void(std::string, float)> cb2 = [](std::string _i2, float _i3) {
    std::cout << "I am cb2, and I have two arguments which values are " << _i2
              << " and " << _i3 << std::endl;
  };

  pol.registerCallback({"i2", "i3"}, cb2);

  auto *o1 = makeOutput<int>("o1");
  auto *o2 = makeOutput<std::string>("o2");
  auto *o3 = makeOutput<float>("o3");

  o1->registerPolicy(&pol, "i1");
  o2->registerPolicy(&pol, "i2");
  o3->registerPolicy(&pol, "i3");

  o1->flush(1);
  o2->flush(std::string("hola"));
  o3->flush(1.0f);

  std::this_thread::sleep_for(std::chrono::seconds(1));
}