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


#include <mico/arduino/flow/ArduinoDeviceBlock.h>
#include <flow/Outpipe.h>
#include <mico/arduino/json.hpp>

#include <sstream>
#include <mico/arduino/SerialPort.h>
#include <string>

namespace mico{
    namespace arduino{
        ArduinoDeviceBlock::ArduinoDeviceBlock(){
            createPipe<bool>("D2");
            createPipe<bool>("D3");
            createPipe<bool>("D4");
            createPipe<bool>("D5");
            createPipe<bool>("D6");
            createPipe<bool>("D7");
            createPipe<std::string>("SoftwareSerial1");
            createPipe<std::string>("SoftwareSerial2");
            createPipe<int>("A0");
            createPipe<int>("A1");
            createPipe<int>("A2");
            createPipe<int>("A3");
            createPipe<int>("A4");
            createPipe<int>("A5");

            createPolicy({  flow::makeInput<bool>("D2"),
                            flow::makeInput<bool>("D3"),
                            flow::makeInput<bool>("D4"),
                            flow::makeInput<bool>("D5"),
                            flow::makeInput<bool>("D6"),
                            flow::makeInput<int>("PWM0"),
                            flow::makeInput<int>("PWM1"),
                            flow::makeInput<int>("PWM2"),
                            flow::makeInput<std::string>("SoftwareSerial1"),
                            flow::makeInput<std::string>("SoftwareSerial2")});

            registerCallback({"D2"}, 
                [&](flow::DataFlow _data){
                    if(arduino_->isOpen()){
                        while(isBeingUsed_){}
                        isBeingUsed_ = true;
                        nlohmann::json  msg;
                        msg["D2"]["is_input"] = 0;
                        msg["D2"]["value"] = (bool)_data.get<bool>("D2");
                        arduino_->writeString(  msg.dump() + "\n");
                        isBeingUsed_ = false;
                    }
                }
            );

            registerCallback({"D3"}, 
                [&](flow::DataFlow _data){
                    if(arduino_->isOpen()){
                        while(isBeingUsed_){}
                        isBeingUsed_ = true;
                        nlohmann::json  msg;
                        msg["D3"]["is_input"] = 0;
                        msg["D3"]["value"] = (bool)_data.get<bool>("D3");
                        arduino_->writeString(  msg.dump() + "\n");
                        isBeingUsed_=false;
                    }
                }
            );
        
            registerCallback({"D4"}, 
                [&](flow::DataFlow _data){
                    if(arduino_->isOpen()){
                        while(isBeingUsed_){}
                        isBeingUsed_ = true;
                        nlohmann::json  msg;
                        msg["D4"]["is_input"] = 0;
                        msg["D4"]["value"] = (bool)_data.get<bool>("D4");
                        arduino_->writeString(  msg.dump() + "\n");
                        isBeingUsed_ = false;
                    }
                }
            );
        
        
            registerCallback({ "PWM0" },
                    [&](flow::DataFlow _data) {
                    auto v = _data.get<int>("PWM0");
                    std::string val = std::to_string(v);
                    if (arduino_->isOpen()) {
                        while (isBeingUsed_) {}
                        isBeingUsed_ = true;
                        arduino_->writeString("{\"PWM0\":" + val + "}\n");
                        isBeingUsed_ = false;
                    }
                }
            );


            registerCallback({ "PWM1" },
                [&](flow::DataFlow _data) {
                auto v = _data.get<int>("PWM1");
                std::string val = std::to_string(v);
                if (arduino_->isOpen()) {
                    while (isBeingUsed_) {}
                    isBeingUsed_ = true;
                    arduino_->writeString("{\"PWM1\":" + val + "}\n");
                    isBeingUsed_ = false;
                }
            }
            );


            registerCallback({ "PWM2" },
                [&](flow::DataFlow _data) {
                auto v = _data.get<int>("PWM2");
                std::string val = std::to_string(v);
                if (arduino_->isOpen()) {
                    while (isBeingUsed_) {}
                    isBeingUsed_ = true;
                    arduino_->writeString("{\"PWM2\":" + val + "}\n");
                    isBeingUsed_ = false;
                }
            }
            );

        }

        ArduinoDeviceBlock::~ArduinoDeviceBlock(){
            isRunning_ = false;
            if(readThread_.joinable())
                readThread_.join();

            if(arduino_){
                arduino_->close();
            }
        }

        bool ArduinoDeviceBlock::configure(std::vector<flow::ConfigParameterDef> _params) {
            if(isRunning_){
                isRunning_ = false;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if (readThread_.joinable())
                    readThread_.join();

                if (arduino_)
                    arduino_->close();
        
            }

            if(auto param = getParamByName(_params, "device"); param){
                try{
                    arduino_ = std::shared_ptr<SerialPort>(new SerialPort(param.value().asString(), 115200));
                }catch(std::exception &_e){
                    std::cout << _e.what();
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
                isRunning_ = true;
                readThread_ = std::thread(&ArduinoDeviceBlock::readLoop, this);
            }

            return true;
        }
    
        std::vector<flow::ConfigParameterDef> ArduinoDeviceBlock::parameters(){
            getListOfDevices();

            #if defined(WIN32)
                std::vector<std::string> defaultPorts = getListOfDevices();
            #elif defined(__linux__)
                std::vector<std::string> defaultPorts = { "/dev/ttyUSB0" };
            #endif

            return {
                {"device", flow::ConfigParameterDef::eParameterType::OPTIONS, defaultPorts}
            };
        }

        void ArduinoDeviceBlock::readLoop(){
            while (isRunning_) {
                std::string cmd = arduino_->readLine();
                //std::cout << cmd << std::endl;
                try {
                    nlohmann::json json = nlohmann::json::parse(cmd);
                    for (nlohmann::json::iterator it = json.begin(); it != json.end(); it++) {
                        if (it.key()[0] == 'D' && getPipe(it.key()) != nullptr) {
                            getPipe(it.key())->flush(it.value().get<bool>());
                        }
                        else if (it.key()[0] == 'A' && getPipe(it.key()) != nullptr) {
                            getPipe(it.key())->flush(it.value().get<int>());
                        }
                        else if (it.key().find("PWM") != std::string::npos && getPipe(it.key()) != nullptr) {

                        }
                        else if (it.key().find("SERIAL") != std::string::npos && getPipe(it.key()) != nullptr) {

                        }
                    }
                }
                catch (std::exception& _e) {

                }
            }
        }
    
        std::vector<std::string> ArduinoDeviceBlock::getListOfDevices() {
        #if defined(_WIN32)
                std::vector<std::string> devices;
                char lpTargetPath[5000]; // buffer to store the path of the COMPORTS
                bool gotPort = false; // in case the port is not found

                for (int i = 0; i < 255; i++) // checking ports from COM0 to COM255
                {
                    std::string str = "COM" + std::to_string(i); // converting to COM0, COM1, COM2
                    DWORD test = QueryDosDevice(str.c_str(), lpTargetPath, 5000);

                    // Test the return value and error if any
                    if (test != 0) { //QueryDosDevice returns zero if it didn't find an object
                        devices.push_back(str);
                        std::cout << str << " --> " << lpTargetPath << std::endl;
                    }

                    if (::GetLastError() == ERROR_INSUFFICIENT_BUFFER) {

                    }
                }

                return devices;
        #endif
        #if defined(__linux__)
            return {};
        #endif
        }
    }
}