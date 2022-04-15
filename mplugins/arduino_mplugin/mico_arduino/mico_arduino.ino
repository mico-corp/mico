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


#include "ArduinoJson.h"
#include "DeviceManager.h"
#include <vector>

StaticJsonDocument<128> doc;
bool getJsonIfAvailable();

void setup() {
  Serial.begin( 115200 ); // Instead of 'Serial1'
}

void loop() {
  // put your main code here, to run repeatedly:
  getJsonIfAvailable();
  DeviceManager::runDevices();
  delay(30);
}

bool getJsonIfAvailable() {
  if (Serial.available()) {
    String json = "";
    int c = '\0';
    while (c != '\n') {
      c = Serial.read();
      Serial.print((char)c);
      if (c == -1)
        break;
      else
        json += char(c);
    }
    
    DeserializationError error = deserializeJson(doc, json);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return false;
    } else {
      JsonObject jsonObj = doc.as<JsonObject>();
      int type = jsonObj["type"];
      switch (type) {
        case 0: // Register
          DeviceManager::registerDevice(jsonObj);
          break;
        case 1: // unregister
          DeviceManager::unregisterDevice(jsonObj);
          break;
        case 2: // Write data
          DeviceManager::processMessage(jsonObj);
        default:
          break;
      }
    }
  } else {
    return false;
  }
}
