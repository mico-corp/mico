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

#include "DevicePin.h"
#include "sstream"

DevicePin *DevicePin::createPin(const std::string &_id, std::string _config){
    int idx = _config.find(',');
    if(idx == _config.npos) return nullptr;

    int pin =  atoi(_config.substr(0,idx).c_str());
    _config = _config.substr(idx+1);
    idx = _config.find(',');
    if(idx == _config.npos) return nullptr;
    bool isAnalog =  atoi(_config.substr(0,idx).c_str());
    bool isOut =  atoi(_config.substr(idx+1).c_str());

    if(isAnalog)
        pin = getAnalogPin(pin);
    
    if(registerPin(pin)){
        return new DevicePin(_id, pin, isAnalog, isOut);
    }else{
        return nullptr;
    }
}

void DevicePin::deinitialize(){
   unregisterPin(pin_);
}

void DevicePin::execute(){
  if(isAnalog_){
      std::stringstream ss;
      ss << "2@" << id_<<"@"<<analogRead(getAnalogPin(pin_));
      Serial.println(ss.str().c_str());
  }else{
    if(isOut_){
      std::stringstream ss;
      ss << "2@" << id_<<"@"<<digitalRead(pin_);
      Serial.println(ss.str().c_str());
    }
  }
}

void DevicePin::process(const std::string &_data) {
  if(!isOut_){
    bool level = atoi(_data.c_str());
    digitalWrite(pin_, level);
  }
}

int DevicePin::getAnalogPin(int _pinIdx){
  switch(_pinIdx){
    case 0:
      return A0;
    case 1:
      return A1;
    case 2:
      return A2;
    case 3:
      return A3;
    case 4:
      return A4;
    case 5:
      return A5;
  }
}

DevicePin::DevicePin(const std::string &_id, int _pin, bool _isAnalog, bool _isOut){
    id_ = _id;
    isOut_ = _isOut;
    isAnalog_ = _isAnalog;
    pin_ = _pin;
    if(!isAnalog_){   
      if(isOut_)
        pinMode(pin_, OUTPUT);
      else
        pinMode(pin_, INPUT);
    }
}
