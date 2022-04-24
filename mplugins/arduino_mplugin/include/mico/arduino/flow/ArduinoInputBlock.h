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



#ifndef MICO_ARDUINO_FLOW_ARDUINOINPUTBLOCK_H_
#define MICO_ARDUINO_FLOW_ARDUINOINPUTBLOCK_H_

#include <flow/Block.h>


class QLineEdit;

namespace mico{
    namespace arduino{

        /// Mico block is used to connecto a physical arduino. With this block, it is possible to send and receive signals
        /// from the arduino for any purpose.
        /// @ingroup  mico_arduino
        ///
        /// @image html blocks/arduino/arduino_block_arduino_input.png width=480px
        ///
        /// __inputs__:
        ///     * data: data to be sent to the arduino. It can be int, float, std::vector<int> or std::vector<float>.
        /// __parameters__:
        ///     * MsgID: User selected ID to me packed with the input data.
        ///
        class ArduinoInputBlock:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Arduino Input";} 

            /// Retreive icon of block    
            QIcon icon() const override { 
                return QIcon((flow::Persistency::resourceDir()/"arduino"/"arduino_input.png").string().c_str());
            }

            /// Base constructor
            ArduinoInputBlock();

            /// Base destructor
            ~ArduinoInputBlock();

            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "\n";};
        private:
            int id_ = -1;
        };

    }

}



#endif