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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_BlockQCustomPlot_H_
#define MICO_FLOW_STREAMERS_BLOCKS_BlockQCustomPlot_H_

#include <flow/Block.h>
#include <opencv2/opencv.hpp>
#include <mutex>

class QLabel;
class QTimer;
class QCustomPlot;

namespace mico{
   
    namespace visualizer{
    
        /// Mico block for visualizing continuous numberical streams of data.
        /// @ingroup  mico_visualizer
        ///
        /// @image html blocks/visualizers/visualizers_block_plotter.png width=480px
        ///
        /// __Inputs__:
        ///     * signal1: 1D signal to be plotted overtime.
        ///     * signal2: 1D signal to be plotted overtime.
        ///     * signal3: 1D signal to be plotted overtime.
        ///
        class BlockQCustomPlot: public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Plotter";}

            /// Base constructor
            BlockQCustomPlot();

            /// Base destructor
            ~BlockQCustomPlot();

            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Signal plotters.\n"
                                                                "   - Inputs: \n";};

        private:
            void realTimePlot();

        private:
            std::mutex dataLock_;
            std::vector<std::pair<float, float>> pendingData1_, pendingData2_, pendingData3_; // 666 ugly but fastly implemented
            QCustomPlot *plot_ = nullptr;
            QTimer *dataTimer_ = nullptr;
            std::mutex imgLock_;

            std::chrono::steady_clock::time_point t0_;
        };
    }
}

#endif
