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

#ifdef MICO_IS_RASPBIAN

#ifndef MICO_ARDUINO_FLOW_RASPBERRYGPIOBLOCK_H_
#define MICO_ARDUINO_FLOW_RASPBERRYGPIOBLOCK_H_

#include <flow/Block.h>


namespace mico{
    namespace arduino{
        /// Mico block is used to connecto a physical arduino. With this block, it is possible to send and receive signals
        /// from the arduino for any purpose.
        /// @ingroup  mico_arduino
        class RaspberryGpioBlock:public flow::Block{
        public:
            /// Get name of block
            virtual std::string name() const override {return "Raspberry gpio";} 

            /// Base constructor
            RaspberryGpioBlock();

            /// Returns a brief description of the block
            std::string description() const override {return    "Low-level access to gpio in Raspberry pi. \n";};

        private:

        };

    }

}



#endif
#endif