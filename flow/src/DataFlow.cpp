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

#include <flow/DataFlow.h>
#include <thread>
#include <flow/ThreadPool.h>

namespace flow{

    #if defined(__linux__)
        std::map<std::string, std::map<std::string, std::function<boost::any(boost::any&)>>> DataFlow::conversions_ = {};
    #endif 

    DataFlow::DataFlow(std::map<std::string, std::string> _flows, std::function<void(DataFlow _f)> _callback){
        callback_ = _callback;
        for(auto &f:_flows){
            types_[f.first] = f.second;
            data_[f.first] = boost::any();
            updated_[f.first] = false;
        }
        lastUsageT_ = std::chrono::system_clock::now();
    }

    void DataFlow::update(std::string _tag, boost::any _data){
        if(data_.find(_tag)!= data_.end()){
            // Can we check here the type?
            data_[_tag] = _data;
            updated_[_tag] = true;
            checkData();
        }else{
            std::invalid_argument("Bad tag type while updating Dataflow");
        }
    }

    void DataFlow::checkData(){
        int flagCounter = 0;
        for(auto flag = updated_.begin(); flag != updated_.end(); flag++){
            if(flag->second) flagCounter++;
        }
        if(flagCounter == updated_.size()){
            ThreadPool::get()->emplace(std::bind(callback_, *this));
            // std::thread(callback_, *this).detach(); // 666 Smthg is not completelly thread safe and produces crash
            auto currT = std::chrono::system_clock::now();
            float incT = std::chrono::duration_cast<std::chrono::microseconds>(currT-lastUsageT_).count();
            lastUsageT_ = currT;
            usageFreq_ = 1/(incT*1e-6);
        }
    }


    float DataFlow::frequency() const{
        return usageFreq_;
    }


    bool DataFlow::checkIfConversionAvailable(std::string const &_from, std::string const &_to){
        if(_from == _to)
            return true;

        if(conversions_.find(_from) != conversions_.end()){
            if(conversions_[_from].find(_to) != conversions_[_from].end()){
                return true;
            }
        }

        if( strcmp(typeid(boost::any).name(), _to.c_str()) == 0 ||
            strcmp(typeid(boost::any).name(), _from.c_str()) == 0){ 
            return true;
        }

        return false;
    }
}