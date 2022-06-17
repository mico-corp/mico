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

namespace flow{
    class Outpipe;


    /// Base class of flow that represents a single input stream.
    /// @ingroup  flow
    class FLOW_DECL PolicyInput{
    public:
        /// Build an input stream with a given name and type
        PolicyInput(std::string _tag, std::string _type) :tag_(_tag), typeName_(_type) {};

        /// Get stream name
        std::string tag(){return tag_;};
        
        /// Get stream type
        std::string typeName(){return typeName_;};
    protected:
        std::string tag_;
        std::string typeName_;
    };

    template<typename T_>
    PolicyInput* makeInput(std::string const &_tag){
        return new PolicyInput(_tag, typeid(T_).name());
    }


    class FLOW_DECL Policy{
        public:
            typedef std::vector<std::string> PolicyMask;

            Policy(std::vector<PolicyInput*> _inputs);

            template<typename ...Arguments>
            bool registerCallback(PolicyMask _mask, std::function<void(Arguments..._args)> _callback);

            void update(std::string _tag, boost::any _data);
    
            int nInputs();
            std::vector<std::string> inputTags();

            std::string type(std::string _tag);

            void associatePipe(std::string _tag, Outpipe* _pipe);

            bool disconnect(std::string _tag);

            std::vector<float> masksFrequencies() const;

        private:
            std::map<std::string, std::string> inputs_;
            std::vector<DataFlow*> flows_;
            std::vector<std::string>                    tags_;
            
            std::unordered_map<std::string, Outpipe*>   connetedPipes_; 
    };
}

namespace flow {

    template<typename ...Arguments>
    bool Policy::registerCallback(PolicyMask _mask, std::function<void(Arguments..._args)> _callback){
        std::map<std::string, std::string> flows;
        for (auto& m : _mask) {
            // auto iter = std::find_if(inputs_.begin(), inputs_.end(), [&](const std::pair<std::string, std::string>& _in){return _in.first == m;});
            auto iter = inputs_.find(m);
            if (iter != inputs_.end()) {
                flows[iter->first] = iter->second;
            }
        }
        if (flows.size()) {
            flows_.push_back(DataFlow::create(flows, _callback));
            return true;
        }
        else {
            return false;
        }
    }
}



#endif