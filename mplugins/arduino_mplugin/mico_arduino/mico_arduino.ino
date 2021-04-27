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


#include <ArduinoJson.h>
#include "CmdRunner.h"
#include "CmdParser.h"

StaticJsonDocument<200> doc;
mico::CmdRunner runner;

bool getJsonIfAvailable();

void setup() {
  // put your setup code here, to run once:
  runner.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  getJsonIfAvailable();
  runner.sendInputs();
  delay(30);
}

bool getJsonIfAvailable(){
  if(Serial.available()){
    String json ="";
    char c = '\0';
    while(c != '\n'){
      c = Serial.read();
      json += c;
    }
    //Serial.print(json);
    
    DeserializationError error = deserializeJson(doc, json);
    if (error) {
      //Serial.print(F("deserializeJson() failed: "));
      //Serial.println(error.f_str());
      return;
    }

  auto cmds = mico::CmdParser::parseMessage(doc);
  for(const auto&cmd:cmds){
    runner.run(cmd);
  }

    return true;
  }else{
    return false;
  }
}
