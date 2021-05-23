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



#ifndef MICO_FLOW_BLOCKS_BLOCKVECTORSPLITTER_H_
#define MICO_FLOW_BLOCKS_BLOCKVECTORSPLITTER_H_

#include <flow/Block.h>

class QSpinBox;

namespace mico{
    namespace core{
        /// Mico block that splits a vector type signal into X signals.
        /// @ingroup  mico_core
        class BlockVectorSplitter:public flow::Block{
        public:
            /// Get name of block
            virtual std::string name() const override {return "Vector Splitter";}        

            /// Base constructor
            BlockVectorSplitter();

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override {return    "BlockVectorSplitter"
                                                                "   - Outputs: \n";};

            QBoxLayout * creationWidget() override;

        private:
            void preparePolicy();
            unsigned nTrajs_ = 1;

            QSpinBox * spinBox_;
        };
    }
}



#endif