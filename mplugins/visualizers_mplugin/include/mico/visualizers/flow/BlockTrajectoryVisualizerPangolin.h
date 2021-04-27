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

#ifdef MICO_HAS_PANGOLIN

#ifndef MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_BLOCKTRAJECTORYVISUALIZERPANGOLIN_H_
#define MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_BLOCKTRAJECTORYVISUALIZERPANGOLIN_H_

#include <flow/Block.h>
#include <mico/visualizers/PangolinVisualizer.h>
#include <QSpinBox>

namespace mico{
    namespace visualizer{
    
        /// Mico block for visualizing 3d trayectories using VTK pcl visualizer.
        /// @ingroup  mico_visualizer
        class BlockTrajectoryVisualizerPangolin: public flow::Block {
        public:
            /// Get name of block
            virtual std::string name() const override {return "Pangolin Trajectory Visualizer";}

            /// Base constructor
            BlockTrajectoryVisualizerPangolin();

            /// Base destructor
            ~BlockTrajectoryVisualizerPangolin();

            virtual /// Get custom view widget to be display in the graph
        QWidget * customWidget() override;
            virtual QBoxLayout * creationWidget() override;

        private:
            void poseCallback(flow::DataFlow  _data, int _id);

            void preparePolicy();

        private:
            unsigned nTrajs_ = 1;

            QSpinBox * spinBox_;

            std::vector<Eigen::Vector3f> lastPositions_;
            std::vector<Eigen::Vector4f> colorLines_ = {{0.0f, 1.0f, 0.0f, 0.6f}, 
                                                        {1.0f, 0.0f, 0.0f, 0.6f}, 
                                                        {0.0f, 0.0f, 1.0f, 0.6f}, 
                                                        {0.6f, 0.6f, 0.0f, 0.6f}, 
                                                        {0.6f, 0.0f, 0.6f, 0.6f}, 
                                                        {0.0f, 0.6f, 0.6f, 0.6f}};
            std::vector<bool> isFirst_;
            
            PangolinVisualizer *visualizer_ = nullptr;

        };
    }

}

#endif
#endif