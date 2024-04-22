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

#include <mico/arduino/flow/ArduinoOutputBlock.h>
#include <mico/arduino/flow/ArduinoConnection.h>
#include <mico/arduino/flow/ArduinoConnectionBlock.h>
#include <QMessageBox>

namespace mico {
    namespace arduino {

        ArduinoOutputBlock::ArduinoOutputBlock() {
            createPipe<boost::any>("output");
        }

        bool ArduinoOutputBlock::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto param = getParamByName(_params, "MsgID"); param) {
                try {
                    id_ = param.value().asInteger();
                    if (!ArduinoConnectionBlock::get()) {
                        //QMessageBox::warning(nullptr, "Arduino not connected", "Please, connect the arduino block first.");
                        return false;
                    }
                    ArduinoConnectionBlock::get()->registerCallback(id_, [&](boost::any& _data) {
                        if (getPipe("output")->registrations()) {
                            getPipe("output")->flush(_data);
                        }
                    });
                }
                catch (std::exception& _e) {
                    std::cout << _e.what();
                    return false;
                }
            }
            return true;
        }

        std::vector<flow::ConfigParameterDef> ArduinoOutputBlock::parameters() {
            return {
                {"MsgID", flow::ConfigParameterDef::eParameterType::INTEGER, 0}
            };
        }

    }
}