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
#include <mico/core/flow/BlockSwitchFlow.h>

#include <cmath>

#include <sstream>

namespace mico {
namespace core {
BlockSwitchFlow::BlockSwitchFlow() {
  createPipe<boost::any>("output");

  createPolicy({flow::makeInput<boost::any>("A"),
                flow::makeInput<boost::any>("B"),
                flow::makeInput<float>("condition")});

  registerCallback({"A", "B", "condition"}, &BlockSwitchFlow::callback, this);
}

void BlockSwitchFlow::callback(boost::any _a, boost::any _b, bool _condition) {
  auto condition = _condition;
  if (condition) {
    getPipe("output")->flush(_a);
  } else {
    getPipe("output")->flush(_b);
  }
}
} // namespace core
} // namespace mico
