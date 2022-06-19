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


#ifndef FLOW_POLICYINPUT_H_
#define FLOW_POLICYINPUT_H_

#include <flow/Export.h>
#include <string>


namespace flow {


    /// Base class of flow that represents a single input stream.
    /// @ingroup  flow
    class PolicyInput {
    public:
        /// Build an input stream with a given name and type
        FLOW_DECL PolicyInput(std::string _tag, std::string _type, bool _isConsumable = false) : tag_(_tag), typeName_(_type), isConsumable_(_isConsumable) {};

        /// Get stream name
        FLOW_DECL std::string tag() const { return tag_; } ;

        /// Get stream type
        FLOW_DECL std::string typeName() const { return typeName_; };

        /// If input is consumable, the DataFlow will wait until new data to call the callback, if not, the data will be reused
        FLOW_DECL bool isConsumable() const { return isConsumable_; };


    protected:
        std::string tag_ = "";
        std::string typeName_ = "";
        bool isConsumable_ = false;
    };

    template<typename T_>
    PolicyInput makeInput(std::string const& _tag, bool _isConsumable = false) {
        return PolicyInput(_tag, typeid(T_).name(), _isConsumable);
    }
}

#endif