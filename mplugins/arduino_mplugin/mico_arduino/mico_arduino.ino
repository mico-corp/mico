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

#include "MessageDealer.h"

void setup() {
  MicoChannel.init();
}

void loop() {
  float f;
  std::vector<float> vf;
  MicoChannel.pull();
  if(MicoChannel.readFrom(1, f)){
    Serial.print("Received data: ");
    Serial.println(f);
    MicoChannel.sendTo(5, f);
  }  
  if(MicoChannel.readFrom(3, vf)){
    Serial.print("Received data: ");
    for(auto &e:vf){
      Serial.print(e);
      Serial.print(", ");
    }
      Serial.print("\n");
    MicoChannel.sendTo(7, vf);
  }
  delay(5);
}
