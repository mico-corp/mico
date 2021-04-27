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



#ifndef MICO_ARDUINO_FLOW_BLOCKJOYPAD_H_
#define MICO_ARDUINO_FLOW_BLOCKJOYPAD_H_

#include <flow/Block.h>
#include <flow/Outpipe.h>

#include <mico/arduino/flow/JoyPad.h>

namespace mico{
    namespace arduino {
        /// Mico interactive block that simulates a joypad to generate -+1 signals in two axis
        /// @ingroup  mico_arduino
        class BlockJoyPad:public flow::Block{
        public:
            /// Retreive name of block;
            std::string name() const override {return "Joy Pad";}     

            /// Get custom view widget to be display in the graph
            QWidget * customWidget() override;
            
            /// Base constructor
            BlockJoyPad();

            /// Returns a brief description of the block
            std::string description() const override {return    "Joy Pad\n";};
        private:
            JoyPad *joypad;
        };
    }
}



#endif