//---------------------------------------------------------------------------------------------------------------------
//  FLOW
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

#include <flow/Policy.h>

#include <flow/Outpipe.h>

#include <cassert>
#include <stdexcept>


namespace flow{

    Policy::Policy(std::vector<PolicyInput*> _inputs){
        if(_inputs.size() == 0){
            throw std::invalid_argument( "A Policy cannot be constructed with an empty list of input pipe tags." );
        }

        for(auto &input:_inputs){
            if(input->tag() == "" || input->typeName() == ""){
                throw std::invalid_argument( "A Policy cannot be constructed with an empty list of input pipe tags." );
            }
            inputs_[input->tag()] = input->typeName();
            tags_.push_back(input->tag());
        }
    }

    bool Policy::registerCallback(PolicyMask _mask, PolicyCallback _callback){
        std::map<std::string, std::string> flows;
        for(auto &m:_mask){
            // auto iter = std::find_if(inputs_.begin(), inputs_.end(), [&](const std::pair<std::string, std::string>& _in){return _in.first == m;});
            auto iter = inputs_.find(m);
            if(iter != inputs_.end()){
                flows[iter->first] = iter->second;
            }
        }
        flows_.push_back(new DataFlow(flows, _callback));
        return true;
    }

    void Policy::update(std::string _tag, boost::any _data){
        for(auto flow:flows_){
            flow->update(_tag, _data);
        }
    }

    int Policy::nInputs(){
        return tags_.size();
    }

    std::vector<std::string> Policy::inputTags(){
        return tags_;
    }

    std::string Policy::type(std::string _tag){
        return inputs_[_tag];
    }

    void Policy::associatePipe(std::string _tag, Outpipe* _pipe){
        connetedPipes_[_tag] = _pipe;
    }

    bool Policy::disconnect(std::string _tag){
        if(auto pipe = connetedPipes_[_tag]; pipe != nullptr){
            pipe->unregisterPolicy(this);
            return true;
        }else{
            return false;
        }
    }


    std::vector<float> Policy::masksFrequencies() const{
        std::vector<float> freqs;
        for(auto &flow: flows_){
            freqs.push_back(flow->frequency());
        }
        return freqs;
    }

}