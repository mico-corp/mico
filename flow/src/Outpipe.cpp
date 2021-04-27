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

#include <flow/Outpipe.h>

#include <flow/Policy.h>

#include <cassert>

#include <stdexcept>

namespace flow{
    Outpipe::Outpipe(std::string _tag, std::string _type):tag_(_tag), type_(_type){
        if(_tag == "" || _type == ""){
            throw std::invalid_argument( "Tag cannot be an empty string" );
        }
    };

    std::string Outpipe::tag() const {return tag_;};
    std::string Outpipe::type() const{return type_;};
    
    bool Outpipe::registerPolicy(Policy * _pol, std::string _policyTag){
        // Check that policy has the output tag
        auto tags = _pol->inputTags();
        auto iter = std::find(tags.begin(), tags.end(), _policyTag);
        if(iter != tags.end()) {
            // Check if policy has ven already registered
            auto iterPol = std::find(registeredPolicies_.begin(), registeredPolicies_.end(), _pol);
            if(iterPol == registeredPolicies_.end()){
                // Check that types are compatible
                // if(type_ == _pol->type(_policyTag)){   // 666 TODO
                if(DataFlow::checkIfConversionAvailable(type_, _pol->type(_policyTag))){
                    policiesGuard.lock();
                    registeredPolicies_.push_back(_pol);
                    tagTranslators_[_pol] = _policyTag;
                    _pol->associatePipe(tagTranslators_[_pol], this);
                    policiesGuard.unlock();
                    return true;
                }
            }
        }
        return false;
    }
    
    void Outpipe::unregisterPolicy(Policy* _pol){
        auto iter = std::find(registeredPolicies_.begin(), registeredPolicies_.end(), _pol);
        if(iter != registeredPolicies_.end()){
            policiesGuard.lock();
            std::cout << "disconnecting pipe" << std::endl;
            registeredPolicies_.erase(iter);
            policiesGuard.unlock();
        }
    }

    void Outpipe::flush(boost::any _data){
        policiesGuard.lock();
        for(auto p:registeredPolicies_){
            p->update(tagTranslators_[p], _data);
        }
        policiesGuard.unlock();
    }

    int Outpipe::registrations(){
        return registeredPolicies_.size();
    }

}
