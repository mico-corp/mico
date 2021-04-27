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

#include "CmdParser.h"

namespace mico{
  
  std::vector<Command> CmdParser::parseMessage(StaticJsonDocument<200> &_data){
    std::vector<Command> cmds;
    JsonObject root = _data.as<JsonObject>();
    for (const JsonPair& keyValue: root) {
      Command cmd = CmdParser::parseCmd(keyValue);
      cmds.push_back(cmd);
    }
    
    return cmds;
  }
      
  Command CmdParser::parseCmd(const JsonPair&  _cmd){
    Command cmd;
    cmd.content_ = _cmd.value();
    String key = _cmd.key().c_str();
    if(key[0] == 'D'){
      cmd.type_ = CMD_TYPE::DIGITAL;
      cmd.pin_ = key[1] - '0';
      //Serial.print("Digital cmd with pin ");
      //Serial.print(cmd.pin_);
      //Serial.print(" and value ");
      //Serial.println(cmd.content_.as<bool>());
    }else if(key[0] == 'A'){
      cmd.type_ = CMD_TYPE::ANALOG;
      cmd.pin_ = key[1] - '0';
      //Serial.print("Analog cmd with pin ");
      //Serial.print(cmd.pin_);
      //Serial.print(" and value ");
      //Serial.println(cmd.content_.as<int>());
    }else if(key.indexOf("PWM") > -1){
      cmd.type_ = CMD_TYPE::PWM;
      cmd.pin_ = key[3] - '0' + 9;
      //Serial.print("Pwm cmd with pin ");
      //Serial.print(cmd.pin_);
      //Serial.print(" and value ");
      //Serial.println(cmd.content_.as<int>());
    }else{  // 
      //Serial.print("None cmd with key ");
      //Serial.print(key);
      //Serial.print(" and content ");
      //Serial.println(cmd.content_.as<int>());
      cmd.type_ = CMD_TYPE::NONE;
    }

    return cmd;
    
  }
      
}
