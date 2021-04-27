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

#include "CmdRunner.h"

#include <Arduino.h>

namespace mico{
  void CmdRunner::init(){
    Serial.begin(115200);
    while (!Serial) continue;
    
    digitalInputs_ = {{2,true},
                      {3,true},
                      {4,true},
                      {5,true},
                      {6,true},
                      {7,true},
                      {8,true},
                      {9,true}};
    
    /// PWMS
//    pinMode(9, OUTPUT);
//    pinMode(10, OUTPUT);
//    pinMode(11, OUTPUT);
    pwm0.attach(9);
    pwm1.attach(10);
    pwm2.attach(11);
  }

  void CmdRunner::run(const Command &_cmd){
    switch(_cmd.type_){
      case CMD_TYPE::DIGITAL:
        runDigital(_cmd);
        break;
      case CMD_TYPE::ANALOG:
        runAnalog(_cmd);
        break;
      case CMD_TYPE::PWM:
        runPwm(_cmd);
        break;
      case CMD_TYPE::SOFT_SERIAL:
        runSerial(_cmd);
        break;
    } 
  }
  
  void CmdRunner::sendInputs(){
    StaticJsonDocument<200> doc;

    for(auto &[pin, isInput]:digitalInputs_){
      if(isInput){
        char key[3] = {'D', '0'+pin, '\0'};
        doc[key] = (bool) digitalRead(pin);
      }
    }
    
    doc["A0"] = analogRead(A0);
    doc["A1"] = analogRead(A1);
    doc["A2"] = analogRead(A2);
    doc["A3"] = analogRead(A3);
    doc["A4"] = analogRead(A4);
    doc["A5"] = analogRead(A5);
    
    serializeJson(doc, Serial);
    Serial.println();
  }

  void CmdRunner::runDigital(const Command &_cmd){
    //Serial.println("Running digital");
    if(_cmd.content_["is_input"].as<bool>()){
      digitalInputs_[_cmd.pin_] = true;
      pinMode(_cmd.pin_, INPUT);
    }else{
      digitalInputs_[_cmd.pin_] = false;
      pinMode(_cmd.pin_, OUTPUT);
      bool val = _cmd.content_["value"].as<bool>();
      digitalWrite(_cmd.pin_, val?HIGH:LOW);
    }
  }
  
  void CmdRunner::runAnalog(const Command &_cmd){
    //Serial.println("Running Analog");
    bool val = _cmd.content_.as<int>();
    
  }
  
  void CmdRunner::runPwm(const Command &_cmd){
    int val = _cmd.content_.as<int>();
    if(_cmd.pin_ == 9){
      pwm0.write(val); 
    }else if(_cmd.pin_ == 10){
      pwm1.write(val);
    }else if(_cmd.pin_ == 11){
      pwm2.write(val);
    }
  }
  
  void CmdRunner::runSerial(const Command &_cmd){
    
  }
  
}
