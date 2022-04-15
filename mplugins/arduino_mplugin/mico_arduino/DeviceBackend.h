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

#ifndef MICOARDUINO_DEVICEBACKEND_H_
#define MICOARDUINO_DEVICEBACKEND_H_

#include "ArduinoSTL.h"
#include "ArduinoJson.h"

#include "set"
#include "string"

class DeviceBackend{
public:
    virtual void execute() = 0;
    virtual void deinitialize() = 0;
    virtual void process(JsonObject &_json) = 0;
    
protected:
    static bool registerPin(const String &_pin){
        if(usedPins_.find(_pin) == usedPins_.end()){
            usedPins_.insert(_pin);
            return true;
        }
        else{
            return false;
        }
    }

    static bool unregisterPin(const String &_pin){
        if(usedPins_.find(_pin) != usedPins_.end()){
            usedPins_.erase(_pin);
        }
        return true;
    }

private:
    static std::set<String> usedPins_;
};


#endif
