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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_BlockScatterPlot_H_
#define MICO_FLOW_STREAMERS_BLOCKS_BlockScatterPlot_H_

#include <flow/Block.h>
#include <opencv2/opencv.hpp>
#include <mutex>

class QLabel;
class QTimer;
class QCustomPlot;

namespace mico{
    namespace visualizer{
    
        /// Mico block for visualizing scattered streams of data in 2D.
        /// @ingroup  mico_visualizer
        ///
        /// @image html blocks/visualizers/visualizers_block_scatter.png width=480px
        ///
        /// __Inputs__:
        ///     * x: X coordinate of next point to be plot
        ///     * y: Y coordinate of next point to be plot
        ///
        class BlockScatterPlot: public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Scatter";}

            BlockScatterPlot();

            ~BlockScatterPlot();

            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir() / "visualizers" / "block_scatter_plot.svg").string();
            }


            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Scatter plot.\n"
                                                                "   - Inputs: \n";};

        private:
            void realTimePlot();

        private:
            std::mutex dataLock_;
            std::vector<std::pair<float,float>> pendingData_; // 666 ugly but fastly implemented
            QCustomPlot *plot_ = nullptr;
            QTimer *dataTimer_ = nullptr;
            std::mutex imgLock_;
        };
    }
}

#endif
