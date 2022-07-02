//---------------------------------------------------------------------------------------------------------------------
//  Cameras wrapper MICO plugin
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



#ifndef MICO_FLOW_BLOCKS_CONSTSTREAMER_H_
#define MICO_FLOW_BLOCKS_CONSTSTREAMER_H_

#include <flow/Block.h>

class QLineEdit;

namespace mico{
    namespace math{
        /// Mico block that generates a stream of a constant number
        /// @ingroup  mico_math 
        class ConstStreamer:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Constant";}
            
            /// Base constructor
            ConstStreamer();
            
            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Method to check if the block has auto running callback
            bool hasRunLoop() const override  { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Stream a constant by a given rate."
                                                                "   - Outputs: Constant value\n";};
            

            /// Get custom view widget to be display in the graph
            QWidget * customWidget() override;

        protected:
            virtual void loopCallback() override;

        private:
            float value_ = 0;
            float rate_ = 30; // FPS

            QLineEdit *valueView_;
        };
    }
}



#endif