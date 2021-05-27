//---------------------------------------------------------------------------------------------------------------------
//  Visualizers MICO plugin
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

#ifndef MICO_FLOW_STREAMERS_BLOCKS_BLOCKFREQUENCYCOUNTER_H_
#define MICO_FLOW_STREAMERS_BLOCKS_BLOCKFREQUENCYCOUNTER_H_

#include <flow/Block.h>
#include <chrono>

class QLabel;
class QTimer;

namespace mico {
    namespace visualizer{
    /// Mico block for visualizing streams of numbers.
    /// @ingroup  mico_visualizer
        class BlockFrequencyCounter : public flow::Block {
        public:
            /// Get name of block
            std::string name() const override { return "Frequency Counter"; }

            BlockFrequencyCounter();

            ~BlockFrequencyCounter();

            /// Returns a brief description of the block
            std::string description() const override { return "Simple stream frequency visualizer block.        \n"
                                                            "   - Inputs: any stream    \n"; };

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Get custom view widget to be display in the graph
            QWidget * customWidget();

        private:
            float freq_ = 0;
            int queueIdx_ = 0;
            constexpr static size_t listSize_ = 30;
            float freqList_[listSize_];
            std::chrono::steady_clock::time_point t0_;


            QLabel *textDisplay_;
            QTimer *refreshTimer_;
        };
    }
} // namespace mico

#endif