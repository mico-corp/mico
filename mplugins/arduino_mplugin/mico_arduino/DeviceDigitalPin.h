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

#ifndef MICOARDUINO_DEVICEDIGITALPIN_H_
#define MICOARDUINO_DEVICEDIGITALPIN_H_

#include "DeviceBackend.h"

class DeviceDigitalOutPin: public DeviceBackend{
public:
    static DeviceDigitalOutPin *createPin(const JsonObject &_config);
    inline static std::string backendName() { return "dpo";};
    void execute() override;
    void deinitialize() override;
    void process(JsonObject &_json) override{};

private:
    DeviceDigitalOutPin(const std::string &_id, int _pin);
    int pin_ = 0;
    std::string id_;
};


class DeviceDigitalInPin: public DeviceBackend{
public:
    static DeviceDigitalInPin *createPin(const JsonObject &_config);
    inline static std::string backendName() { return "dpi";};

    void execute() override {};
    void deinitialize() override;
    void process(JsonObject &_json) override;
    
private:
    DeviceDigitalInPin(const std::string &_id, int _pin);
    int pin_ = 0;
    std::string id_;
};


#endif
