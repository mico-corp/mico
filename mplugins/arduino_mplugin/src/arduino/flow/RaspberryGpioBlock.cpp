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

#ifdef MICO_IS_RASPBIAN

#include <flow/Outpipe.h>
#include <mico/arduino/flow/RaspberryGpioBlock.h>
#include <mico/arduino/flow/RaspberryGpioHelperNoSudo.h>

namespace mico {
namespace arduino {
RaspberryGpioBlock::RaspberryGpioBlock() {
  if (gpioInitialise() < 0)
    return;

  // createPipe<bool>("GPIO14");
  createPolicy({flow::makeInput<bool>("GPIO14")});

  gpioSetMode(14, PI_OUTPUT);
  gpioWrite(14, 0);
  registerCallback({"GPIO14"}, [&](flow::DataFlow _data) {
    const unsigned level = (bool)_data.get<bool>("GPIO14") ? 1 : 0;
    gpioWrite(14, level);
  });
}

} // namespace arduino
} // namespace mico

#endif