//-----------------------------------------------------------------------------
//  Arduino MICO plugin
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

#include <flow/flow.h>
#include <flow/plugins/BlockPlugin.h>
#include <mico/raspbian/flow/RaspberryGpioBlock.h>
#include <mico/raspbian/flow/RaspiCam.h>

using namespace mico::raspbian;
using namespace flow;

extern "C" FLOW_FACTORY_EXPORT flow::PluginNodeCreator *
factory(fs::path _libraryPath) {
  Persistency::setResourceDir(_libraryPath.parent_path().string() +
                              "/resources");
  flow::PluginNodeCreator *creator = new flow::PluginNodeCreator;

#ifdef MICO_IS_RASPBIAN
  creator->registerNodeCreator(
      []() { return std::make_shared<RaspberryGpioBlock>(); },
      "Raspberry Gpio");
  #endif
  #ifdef MICO_IS_RASPBIAN_DEPRECATED
  creator->registerNodeCreator([]() { return std::make_shared<RaspiCam, >(); },
                               "cameras");
#endif

  return creator;
}
