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

#include <flow/flow.h>

#include <mico/visualizers/flow/BlockImageVisualizer.h>
#include <mico/visualizers/flow/BlockNumberVisualizer.h>
#include <mico/visualizers/flow/BlockFrequencyCounter.h>
#include <mico/visualizers/flow/BlockPointCloudVisualizer.h>
#include <mico/visualizers/flow/BlockSceneVisualizer.h>
#include <mico/visualizers/flow/BlockQCustomPlot.h>
#include <mico/visualizers/flow/BlockScatterPlot.h>


using namespace mico::visualizer;
using namespace flow;

extern "C" FLOW_FACTORY_EXPORT flow::PluginNodeCreator* factory(fs::path _libraryPath){
    Persistency::setResourceDir(_libraryPath.parent_path().string() + "/resources");
    flow::PluginNodeCreator *creator = new flow::PluginNodeCreator;

    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockNumberVisualizer > >(); },  "Visualizers");
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockImageVisualizer > >(); },   "Visualizers");
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockQCustomPlot > >(); },       "Visualizers");
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockScatterPlot > >(); },       "Visualizers");
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockFrequencyCounter > >(); },  "Visualizers");

    #ifdef HAS_MICO_SLAM
        creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockPointCloudVisualizer> >(); }, "Visualizers");
        creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockSceneVisualizer> >(); }, "Visualizers");
    #endif
    
    return creator;
}
