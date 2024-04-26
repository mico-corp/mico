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

#include "MessageDealer.h"

MessageDealer MicoChannel;

void MessageDealer::pull() {
  String data = "";
  String id = "";
  if (Serial.available()) {
    char c = 0;
    int idx = 0;
    for (;;) {
      c = Serial.read();
      if (c == -1)
        return; // Error
      if (c == '@')
        break; // Got id, read data
      id += c;
    }
    for (;;) {
      c = Serial.read();
      if (c == -1)
        return; // Error
      if (c == '\n') {
        break;
      } else {
        data += c;
      }
    }
    // Serial.print("id: ");
    // Serial.print(id);
    // Serial.print(". Data: ");
    // Serial.println(data);
    data_[atoi(id.c_str())].push(data);
  }
}

bool MessageDealer::isDataAvailable(int _id) {
  return data_.find(_id) != data_.end();
}
