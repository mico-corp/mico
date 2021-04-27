//---------------------------------------------------------------------------------------------------------------------
//  robotics MICO plugin
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


#include <mico/robotics/flow/BlockSerialPort.h>
#include <flow/Outpipe.h>
#include <sstream>
/*
namespace mico{
    
    BlockSerialPort::BlockSerialPort(){
        createPipe("tx", "byte");

        createPolicy({{"rx", "byte"} });

        registerCallback({"rx"}, 
                                [&](flow::DataFlow _data){
                                    auto byte = _data.get<uint8_t>("rx");
                                    std::string msg; msg += (char) byte;
                                    sp_->writeString(msg);     
                                    // write to serial port
                                }
        );
    }


    BlockSerialPort::~BlockSerialPort(){
        run_ = false;
        if(readThread_.joinable())
            readThread_.join();
    }

    bool BlockSerialPort::configure(std::vector<flow::ConfigParameterDef> _params) {
        for(auto &p:_params){
            std::stringstream ss;
            ss << p.second;
            if(p.first == "device"){
                ss >> device_;
            }if(p.first == "baudrate"){
                ss >> baudrate_;
            }
        }
        if(device_ == ""){
            return false;
        }else{
            // Open serial port
            sp_ = new SerialPort(device_, baudrate_);

            readThread_ = std::thread([&](){
                while(run_){
                    uint8_t b = sp_->readByte();
                    getPipe("tx")->flush(b);
                }
            });

            return true;
        }
    }
    
    std::vector<flow::ConfigParameterDef> BlockSerialPort::parameters(){
        return {
            {"device", flow::ConfigParameterDef::eParameterType::STRING},
            {"baudrate", flow::ConfigParameterDef::eParameterType::INTEGER}
        };
    }

}*/