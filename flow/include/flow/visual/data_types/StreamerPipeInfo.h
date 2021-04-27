
//---------------------------------------------------------------------------------------------------------------------
//  flow
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


#ifndef FLOW_VISUAL_DATATYPES_STREAMERPIPEINFO_H_
#define FLOW_VISUAL_DATATYPES_STREAMERPIPEINFO_H_

#include <flow/Export.h>

#include <nodes/NodeDataModel>

#include <cassert>

using QtNodes::NodeData;
using QtNodes::NodeDataType;

namespace flow{
    // Forward declaration
    class Block;
}

namespace flow {

    struct PipeInfo{
        std::string pipeName_ = "";
        std::shared_ptr<flow::Block> otherBlock_ = nullptr;
    };

    class FLOW_DECL StreamerPipeInfo : public NodeData {
    public:
        StreamerPipeInfo() {} 

        StreamerPipeInfo(std::shared_ptr<flow::Block> const &_blockRef, const std::string _pipeName) {
            pipeInfo_.otherBlock_ = _blockRef;
            pipeInfo_.pipeName_ = _pipeName;
        }

        NodeDataType type() const override {
            return NodeDataType{"pipe_info", pipeInfo_.pipeName_.c_str()};
        }

        PipeInfo info() const { 
            return pipeInfo_; 
        }

    private:
        PipeInfo pipeInfo_;
        
    };

} // namespace flow


#endif