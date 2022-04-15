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

#include "DeviceDigitalPin.h"

DeviceDigitalPin *DeviceDigitalPin::createPin(const JsonObject &_config){
    int pin =  _config["config"]["pin"];
    bool isOut_ = _config["config"]["is_out"];
    if(registerPin("D" + ('0'+pin))){
        return new DeviceDigitalPin(_config["id"], pin, isOut_);
    }else{
        return nullptr;
    }
}

void DeviceDigitalPin::deinitialize(){
   unregisterPin("D" + ('0'+pin_));
}

void DeviceDigitalPin::execute(){
  if(isOut_){
    StaticJsonDocument<16> doc;
    doc["id"] = id_;
    doc["data"] = digitalRead(pin_);
    serializeJson(doc, Serial);
    Serial.print('\n');
  }
}

void DeviceDigitalPin::process(JsonObject &_json) {
  if(!isOut_){
    bool level = _json["data"];
    Serial.println(level);
    digitalWrite(pin_, level);
  }
}

DeviceDigitalPin::DeviceDigitalPin(const std::string &_id, int _pin, bool _isOut){
    id_ = _id;
    pin_ = _pin;
    isOut_ = _isOut;
  if(isOut_)
    pinMode(pin_, OUTPUT);
  else
    pinMode(pin_, INPUT);
}
