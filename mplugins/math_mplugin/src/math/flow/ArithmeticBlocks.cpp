//-----------------------------------------------------------------------------
//  Cameras wrapper MICO plugin
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

#include <flow/Outpipe.h>
#include <flow/Policy.h>
#include <mico/math/flow/ArithmeticBlocks.h>

#include <cmath>

#include <sstream>

namespace mico {
namespace math {
BlockSum::BlockSum() {
  createPipe<float>("result");

  createPolicy(
      {flow::makeInput<float>("in_1"), flow::makeInput<float>("in_2")});

  std::function<void(float, float)> fn = [&](float _i1, float _i2) {
    getPipe("result")->flush(_i1 + _i2);
  };
  registerCallback({"in_1", "in_2"}, fn);
}

BlockSubstract::BlockSubstract() {
  createPipe<float>("result");

  createPolicy(
      {flow::makeInput<float>("in_1"), flow::makeInput<float>("in_2")});

  std::function<void(float, float)> fn = [&](float _i1, float _i2) {
    getPipe("result")->flush(_i1 - _i2);
  };
  registerCallback({"in_1", "in_2"}, fn);
}

BlockMultiply::BlockMultiply() {
  createPipe<float>("result");

  createPolicy(
      {flow::makeInput<float>("in_1"), flow::makeInput<float>("in_2")});

  std::function<void(float, float)> fn = [&](float _i1, float _i2) {
    getPipe("result")->flush(_i1 * _i2);
  };
  registerCallback({"in_1", "in_2"}, fn);
}

BlockDivide::BlockDivide() {
  createPipe<float>("result");

  createPolicy(
      {flow::makeInput<float>("in_1"), flow::makeInput<float>("in_2")});

  std::function<void(float, float)> fn = [&](float _i1, float _i2) {
    getPipe("result")->flush(_i1 / _i2);
  };
  registerCallback({"in_1", "in_2"}, fn);
}

BlockSquareRoot::BlockSquareRoot() {
  createPipe<float>("result");

  createPolicy({flow::makeInput<float>("in")});

  std::function<void(float)> fn = [&](float _in) {
    getPipe("result")->flush(sqrt(_in));
  };
  registerCallback({"in_"}, fn);
}

BlockPow::BlockPow() {
  createPipe<float>("result");

  createPolicy({flow::makeInput<float>("base"), flow::makeInput<float>("exp")});

  std::function<void(float, float)> fn = [&](float _i1, float _i2) {
    getPipe("result")->flush(float(pow(_i1, _i2)));
  };
  registerCallback({"base", "exp"}, fn);
}

BlockIntegrator::BlockIntegrator() {
  resetBt_ = new QPushButton("Reset");
  QObject::connect(resetBt_, &QPushButton::clicked, [&]() {
    acc_ = 0.0f;
    getPipe("output")->flush(acc_);
  });

  createPipe<float>("output");

  createPolicy({flow::makeInput<float>("input")});

  std::function<void(float)> fn = [&](float _in) {
    acc_ += _in;
    getPipe("output")->flush(acc_);
  };

  registerCallback({"input"}, fn);
}

BlockDerivative::BlockDerivative() {
  resetBt_ = new QPushButton("Reset");
  QObject::connect(resetBt_, &QPushButton::clicked, [&]() {
    lastVal_ = 0.0f;
    isFirst_ = true;
    t0_ = std::chrono::high_resolution_clock::now();
  });

  createPipe<float>("output");
  createPolicy({flow::makeInput<float>("input")});

  std::function<void(float)> fn = [&](float _in) {
    if (isFirst_) {
      t0_ = std::chrono::high_resolution_clock::now();
      lastVal_ = _in;
      isFirst_ = false;
    } else {
      auto t1 = std::chrono::high_resolution_clock::now();
      float incT =
          std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0_)
              .count();
      t0_ = t1;
      incT /= 1e6;
      float deriv = (_in - lastVal_) / incT;
      getPipe("output")->flush(deriv);

      lastVal_ = _in;
    }
  };
  registerCallback({"input"}, fn);
}
} // namespace math
} // namespace mico