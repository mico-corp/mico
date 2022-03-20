//---------------------------------------------------------------------------------------------------------------------
//   fastcom wrapper MICO plugin
//---------------------------------------------------------------------------------------------------------------------
//   Copyright 2020 - Marco Montes Grova (a.k.a. mgrova) marrcogrova@gmail.com 
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

#include <mico/fastcom_wrapper/flow/streamers/FastcomSubscribers.h>
#include <mico/fastcom_wrapper/flow/publishers/FastcomPublishers.h>

namespace fastcom_wrapper{

    extern "C" FLOW_FACTORY_EXPORT flow::PluginNodeCreator* factory(fs::path _libraryPath){
        Persistency::setResourceDir(_libraryPath.parent_path().string() + "/resources");
        flow::PluginNodeCreator *creator = new flow::PluginNodeCreator;

        creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockFastcomImagePublisher > >(); }, "Fastcom");
        creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockFastcomImageSubscriber > >(); }, "Fastcom");
        
        creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockFastcomFloatPublisher > >(); }, "Fastcom");
        creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockFastcomFloatSubscriber > >(); }, "Fastcom");
        
        creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockFastcomIntPublisher   > >(); }, "Fastcom");
        creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockFastcomIntSubscriber   > >(); }, "Fastcom");
        
        
        // creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockFastcomVectorfSubscriber > >(); }, "Fastcom");
        // creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockFastcomVectorfPublisher  > >(); }, "Fastcom");
        
        return creator;
    }  
}
