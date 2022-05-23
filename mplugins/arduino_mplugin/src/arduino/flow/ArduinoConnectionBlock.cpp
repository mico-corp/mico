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


#include <mico/arduino/flow/ArduinoConnectionBlock.h>
#include <mico/arduino/flow/ArduinoConnection.h>
#include <flow/Outpipe.h>

#include <sstream>
#include <string>

#ifdef _WIN32
    #include <windows.h>
#endif

namespace mico{
    namespace arduino{
        ArduinoConnectionBlock::ArduinoConnectionBlock(){

        }

        ArduinoConnectionBlock::~ArduinoConnectionBlock(){
        }

        bool ArduinoConnectionBlock::configure(std::vector<flow::ConfigParameterDef> _params) {
            if(arduino_){
                arduino_->close();
                arduino_ = nullptr;
            }

            if(auto param = getParamByName(_params, "device"); param){
                try{
                    arduino_ = std::shared_ptr<ArduinoConnection>(new ArduinoConnection(param.value().selectedOption(), 115200));
                }catch(std::exception &_e){
                    std::cout << _e.what();
                    return false;
                }
            }

            return true;
        }
    
        std::vector<flow::ConfigParameterDef> ArduinoConnectionBlock::parameters(){
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

        std::vector<std::string> ArduinoConnectionBlock::getListOfDevices() {
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