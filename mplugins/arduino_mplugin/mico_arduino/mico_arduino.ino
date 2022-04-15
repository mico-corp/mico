//---------------------------------------------------------------------------------------------------------------------
//  Arduino MICO plugin
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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

#include "DeviceManager.h"
#include <vector>

bool getJsonIfAvailable();

void setup() {
  Serial.begin( 115200 ); // Instead of 'Serial1'
}

void loop() {
  // put your main code here, to run repeatedly:
  getJsonIfAvailable();
  DeviceManager::runDevices();
  delay(5);
}

bool getJsonIfAvailable() {
  if (Serial.available()) {
    std::string data = "";
    int c = '\0';
    while (c != '\n') {
      c = Serial.read();
      Serial.print((char)c);
      if (c == -1)
        break;
      else
        data += char(c);
    }

    // Parse data
    int idx = data.find('@');
    if(idx == data.npos) return false;
    int type = atoi(data.substr(0,idx).c_str());
    data = data.substr(idx+1);
    idx = data.find('@');
    if(idx == data.npos) return false;
    std::string id = data.substr(0,idx);
    data = data.substr(idx+1);

    // Run
    switch (type) {
      case 0: // Register
        DeviceManager::registerDevice(id, data);
        break;
      case 1: // unregister
        DeviceManager::unregisterDevice(id);
        break;
      case 2: // Write data
        DeviceManager::processMessage(id, data);
      default:
        break;
    }

  } else {
    return false;
  }
}
