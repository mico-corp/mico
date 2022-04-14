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


#include <mico/arduino/flow/devices/DigitalPinInDevice.h>
#include <mico/arduino/flow/ArduinoConnectionBlock.h>
#include <mico/arduino/flow/ArduinoConnection.h>
#include <mico/arduino/json.hpp>

#include <QHBoxLayout>
#include <QSpinBox>
#include <QLabel>

namespace mico {
    namespace arduino {
        DigitalPinInDevice::DigitalPinInDevice() {
            createPipe<bool>("out");

        }


        bool DigitalPinInDevice::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto param = getParamByName(_params, "pin"); param) {
                auto pin = param.value().asInteger();
                if (pin < 0 || pin>5) {
                    return false;
                }
                else {
                    uninitialize();
                    currentPin_ = pin;
                    initialize();
                }
            }

            return true;
        }

        std::vector<flow::ConfigParameterDef> DigitalPinInDevice::parameters() {
            return {
                {"pin", flow::ConfigParameterDef::eParameterType::INTEGER, 0}
            };
        }



        void DigitalPinInDevice::initialize() {
            deviceId_ = ArduinoConnectionBlock::get()->createUniqueKey();
            nlohmann::json config;
            config["type"] = 0;
            config["id"] = deviceId_;
            config["bknd"] = "dpo";
            config["config"]["pin"] = currentPin_;
            ArduinoConnectionBlock::get()->registerDevice(config, [&](nlohmann::json& _data) {
                getPipe("out")->flush((bool) _data["data"]);
            });
        }

        void DigitalPinInDevice::uninitialize() {
            if (deviceId_ != "") {
                nlohmann::json config;
                config["type"] = 1;
                config["id"] = deviceId_;
                ArduinoConnectionBlock::get()->unregisterDevice(config);
                deviceId_ = "";
            }
        }


    }
}