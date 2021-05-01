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



#ifndef MICO_ARDUINO_FLOW_FUNCTIONBLOCKS_H_
#define MICO_ARDUINO_FLOW_FUNCTIONBLOCKS_H_

#include <flow/Block.h>


class QLineEdit;
class SerialPort;

namespace mico{
    namespace arduino{
        /// Mico block is used to connecto a physical arduino. With this block, it is possible to send and receive signals
        /// from the arduino for any purpose.
        /// @ingroup  mico_arduino
        class ArduinoDeviceBlock:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Arduino Device";} 

            /// Retreive icon of block    
            QIcon icon() const override { 
                return QIcon((flow::Persistency::resourceDir()/"arduino"/"arduino_icon.png").string().c_str());
            }

            /// Base constructor
            ArduinoDeviceBlock();

            /// Base destructor
            ~ArduinoDeviceBlock();

            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Arduino Device. Configure connection with"
                                                                "arduino device to use the rest of the blocks\n";};

        protected:
            void readLoop();

            void parseArduinoMessage();

            std::vector<std::string> getListOfDevices();

        private:
            std::shared_ptr<SerialPort> arduino_;
            bool isBeingUsed_ = false;
            bool isRunning_ = true;
            std::thread readThread_;
        };

    }

}



#endif