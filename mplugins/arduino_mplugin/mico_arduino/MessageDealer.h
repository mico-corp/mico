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

#ifndef MICO_ARDUINO_MESSAGEDEALER_H_
#define MICO_ARDUINO_MESSAGEDEALER_H_

#include <Arduino.h>
#include <ArduinoSTL.h>
#include <map>
#include <queue>
#include <vector>

///
/// Intermadiate class to read and write messages from and to MICO marduino
/// plugin
///
class MessageDealer {
public:
  bool init() { Serial.begin(115200); }

  /// Send data to MICO
  /// \param _id: ID to associate to the message
  /// \param _data: Data to be sent
  template <typename T_> void sendTo(int _id, T_ _data);

  /// Pull data from Serial port
  void pull();

  /// Check if there is any message available from MICO
  /// \return True if there is data False if not
  bool isDataAvailable(int _id);

  /// Read next message with given ID
  /// \param _id: ID to query data from
  /// \param _data: variable to contain data
  /// \return True if succeeded false if not.
  template <typename T_> bool readFrom(int _id, T_ &_data);

private:
  std::map<int, std::queue<String>> data_;
};

#include "MessageDealer.hpp"

#endif
