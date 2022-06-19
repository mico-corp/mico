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


#ifndef FLOW_POLICY_H_
#define FLOW_POLICY_H_

#include <flow/Export.h>

#include <vector>
#include <cstdlib>

#include <boost/any.hpp>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <iostream>
#include <functional>

#include <flow/DataFlow.h>
#include <flow/PolicyInput.h>

namespace flow{
    class Outpipe;

    class Policy{
        public:
            typedef std::vector<std::string> PolicyMask;

            FLOW_DECL Policy(std::vector<PolicyInput> _inputs);

            template<typename ...Arguments>
            bool registerCallback(PolicyMask _mask, std::function<void(Arguments..._args)> _callback);

            template<typename T_, typename ...Arguments>
            bool registerCallback(PolicyMask _mask, void (T_::*_callback)(Arguments..._args), T_ *obj);

            FLOW_DECL void update(std::string _tag, boost::any _data);
    
            FLOW_DECL size_t nInputs();
            FLOW_DECL std::vector<std::string> inputTags();

            FLOW_DECL std::string type(std::string _tag);

            FLOW_DECL void associatePipe(std::string _tag, Outpipe* _pipe);

            FLOW_DECL bool disconnect(std::string _tag);

        private:
            std::vector<PolicyInput>    inputs_;
            std::vector<DataFlow*>      flows_;
            std::vector<std::string>    tags_;
            
            std::unordered_map<std::string, Outpipe*>   connetedPipes_; 
    };
}

namespace flow {

    template<typename ...Arguments>
    bool Policy::registerCallback(PolicyMask _mask, std::function<void(Arguments..._args)> _callback){
        std::vector<PolicyInput> dfInputs;
        for (auto& element : _mask) {
            // auto iter = std::find_if(inputs_.begin(), inputs_.end(), [&](const std::pair<std::string, std::string>& _in){return _in.first == m;});
            auto iter = std::find_if(inputs_.begin(), inputs_.end(), [&element](const PolicyInput& _input) {
                return _input.tag() == element;
            });

            if (iter != inputs_.end()) {
                dfInputs.push_back(*iter);
            }
        }
        if (dfInputs.size()) {
            flows_.push_back(DataFlow::create(dfInputs, _callback));
            return true;
        }
        else {
            return false;
        }
    }


    template<typename T_, typename ...Arguments>
    bool Policy::registerCallback(PolicyMask _mask, void (T_::* _cb)(Arguments..._args), T_* _obj) {
        std::function<void(Arguments..._args)> fn = [_cb, _obj](Arguments..._args) {
            (_obj->*_cb)(_args...);
        };
        return registerCallback(_mask, fn);
    }
}



#endif