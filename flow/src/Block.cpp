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

#include <flow/Block.h>
#include <flow/Outpipe.h>

#include <cassert>

namespace flow{
    // BASE METHODS
    Block::~Block(){
        this->stop();
    }
    
    std::unordered_map<std::string, std::shared_ptr<Outpipe>> Block::getPipes(){
        return opipes_;
    }

    std::shared_ptr<Outpipe> Block::getPipe(std::string _tag){
        if(opipes_.find(_tag) != opipes_.end())
            return opipes_[_tag];
        else
            return nullptr;
    }

    void Block::start(){
        if(!runLoop_){
            runLoop_ = true;
            loopThread_ = std::thread(&Block::loopCallback, this);
        }
    }

    void Block::stop(){
        if(runLoop_){
            runLoop_ = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if(loopThread_.joinable())
                loopThread_.join();
        }
    }

    
    int Block::nInputs(){
        if(iPolicy_)
            return iPolicy_->nInputs();
        else
            return 0;
    }

    std::vector<std::string> Block::inputTags(){
        if(iPolicy_)
            return iPolicy_->inputTags();
        else
            return {};
    }

    int Block::nOutputs(){
        return opipes_.size();
    }

    std::vector<std::string> Block::outputTags(){
        std::vector<std::string> tags;
        for(auto &os: opipes_){
            tags.push_back(os.first);
        }

        return tags;
    }

    Policy* Block::getPolicy(){
        return iPolicy_;
    }

    void Block::connect(std::string _pipeTag, std::string _policyTag, Block& _otherBlock){
        if(opipes_[_pipeTag] != nullptr){
            opipes_[_pipeTag]->registerPolicy(_otherBlock.getPolicy(), _policyTag);
        }
    }

    void Block::disconnect(std::string _pipeTag) {
        iPolicy_->disconnect(_pipeTag);
    }

    
    std::optional<ConfigParameterDef> Block::getParamByName(const std::vector<flow::ConfigParameterDef> &_params, const std::string &_pname){
        auto iter = std::find_if(  _params.begin(), 
                                _params.end(), 
                                [&_pname](const ConfigParameterDef &_param ){ 
                                    return _param.name_ == _pname;
                                });

        if(iter == _params.end()){
            return std::nullopt;
        }else{
            return *iter;
        }
    }

    bool Block::removePipe(std::string _pipeTag){
        opipes_.erase(_pipeTag);

        std::unordered_map<std::string, std::shared_ptr<Outpipe>>::const_iterator got = opipes_.find(_pipeTag);
        if ( got == opipes_.end()){
            return true;
        }else{
            return false;
        }
    }

    bool Block::removePipes(){
        opipes_.clear();

        return opipes_.empty();
    }

    bool Block::isRunningLoop() const{
        return runLoop_;
    }

    bool Block::createPolicy(std::vector<PolicyInput*> _inputs){
        if(iPolicy_){
            return false;
        }else{
            iPolicy_ = new flow::Policy(_inputs);
            return true;
        }
    }

    void Block::removePolicy(){
        if(iPolicy_)
            delete iPolicy_;
        iPolicy_ = nullptr;
    }
        

    bool Block::registerCallback(Policy::PolicyMask _mask, Policy::PolicyCallback _callback){
        if(iPolicy_){
            iPolicy_->registerCallback( _mask,  _callback );
            return true;
        }else{
            return false;
        }
    }

}