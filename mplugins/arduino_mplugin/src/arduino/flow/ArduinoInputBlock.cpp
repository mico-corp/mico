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

#include <mico/arduino/flow/ArduinoInputBlock.h>
#include <mico/arduino/flow/ArduinoConnection.h>
#include <mico/arduino/flow/ArduinoConnectionBlock.h>
#include <QMessageBox>

namespace mico {
	namespace arduino {
        
        ArduinoInputBlock::ArduinoInputBlock() {
            createPolicy({ flow::makeInput<bool>("data") });

            registerCallback({ "data" },
                [&](flow::DataFlow _data) {
                    auto data = _data.get<boost::any>("data");
                    if (strcmp(typeid(bool).name(), data.type().name()) == 0) {  // 666 TODO Not fancy but want to see it working nowwww!
                        ArduinoConnectionBlock::get()->write(id_, (int) boost::any_cast<bool>(data));
                    } else if(strcmp(typeid(int).name(), data.type().name()) == 0) {  // 666 TODO Not fancy but want to see it working nowwww!
                        ArduinoConnectionBlock::get()->write(id_, boost::any_cast<int>(data));
                    }
                    else if (strcmp(typeid(float).name(), data.type().name()) == 0) {
                        ArduinoConnectionBlock::get()->write(id_, boost::any_cast<float>(data));
                    }else if (strcmp(typeid(std::vector<int>).name(), data.type().name()) == 0) {
                        ArduinoConnectionBlock::get()->write(id_, boost::any_cast<std::vector<int>>(data));
                    }else if (strcmp(typeid(std::vector<float>).name(), data.type().name()) == 0) {
                        ArduinoConnectionBlock::get()->write(id_, boost::any_cast<std::vector<float>>(data));
                    }
                }
            );
        }

        
        ArduinoInputBlock::~ArduinoInputBlock() {

        }

        bool ArduinoInputBlock::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (auto param = getParamByName(_params, "MsgID"); param) {
                try {
                    id_ = param.value().asInteger();
                    if (!ArduinoConnectionBlock::get()) {
                        QMessageBox::warning(nullptr, "Arduino not connected", "Please, connect the arduino block first.");
                        return false;
                    }
                }
                catch (std::exception& _e) {
                    std::cout << _e.what();
                    return false;
                }
            }
            return true;
        }

        std::vector<flow::ConfigParameterDef> ArduinoInputBlock::parameters() {
            return {
                {"MsgID", flow::ConfigParameterDef::eParameterType::INTEGER, 0}
            };
        }

	}
}