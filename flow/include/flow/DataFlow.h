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


#ifndef FLOW_DATAFLOW_H_
#define FLOW_DATAFLOW_H_

#include <flow/Export.h>

#include <boost/any.hpp>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <cstring>

namespace flow{
        
    /// Base class of flow that a flow of data. It manages the implicit conversion of data through the streams and call the 
    /// associated callbacks. It is used in input policies to generate automatic data flows.
    /// @ingroup  flow
    class DataFlow{
    public:
        /// Construct a data flow with a set if input flows and associate a callback to it.
        DataFlow(std::map<std::string, std::string> _flows, std::function<void(DataFlow _f)> _callback);

        /// Update an specific input tag. If all the inputs have been satisfied then the callback is called.
        void update(std::string _tag, boost::any _data);

        /// Query to check internal status of data and update the DataFlow
        void checkData();

        /// Templatized method to get data of any type. Get methods are implemented all around the different plugins
        template<typename T_>
        T_ get(std::string const &_tag);

        /// Get the current frequency at which the DataFlow is triggering the callback.
        float frequency() const;

    private:
        std::map<std::string, std::string>  types_;
        std::map<std::string, boost::any>     data_;
        std::map<std::string, bool>         updated_;
        std::function<void(DataFlow _f)> callback_;
        
        std::chrono::time_point<std::chrono::system_clock> lastUsageT_;
        float usageFreq_ = 0;

    public:
        static std::map<std::string, std::map<std::string, std::function<boost::any(boost::any&)>>> conversions_;
        static bool checkIfConversionAvailable(std::string const &_from, std::string const &_to);
    };

}

namespace flow {

    template<typename T_>
    inline T_ DataFlow::get(std::string const &_tag){
        if(types_.find(_tag) != types_.end()){
            //throw std::invalid_argument("Input tag does not exist, Add it as policy");
            if(strcmp(typeid(T_).name(), data_[_tag].type().name()) == 0 ){
                return boost::any_cast<T_>(data_[_tag]);                
            }else{
                if( auto iter = conversions_.find(data_[_tag].type().name()); iter != conversions_.end()){
                    if(iter->second.find(typeid(T_).name()) != iter->second.end()){
                        std::function<boost::any(boost::any&)> fn = iter->second[typeid(T_).name()];
                        return boost::any_cast<T_>(fn(data_[_tag]));
                    }
                }
            }
        } 
        

        if constexpr (std::is_arithmetic_v<T_>)
            return 0;
        else if constexpr (std::is_default_constructible_v<T_>)
            return T_();
        else
            throw std::invalid_argument("Bad tag type when getting data from DataFlow");
    }

    template<>
    inline boost::any DataFlow::get(std::string const& _tag) {
        return data_[_tag];
    }
}                                                                                \


#if defined(WIN32)
    #define INIT_FLOW_CONVERSION_MAP()                                                                                  \
        std::map<std::string, std::map<std::string, std::function<boost::any(boost::any&)>>> flow::DataFlow::conversions_ = {};
#else
    #define INIT_FLOW_CONVERSION_MAP()                             
#endif

#define FLOW_CONVERSION_REGISTER(Type1_, Type2_, conversion_)                                                       \
    namespace flow{                                                                                                 \
        struct ConversionRegistrator##Type1_##Type2_{                                                               \
            ConversionRegistrator##Type1_##Type2_(){                                                                \
                DataFlow::conversions_[typeid(Type1_).name()][typeid(Type2_).name()] = conversion_;                 \
            }                                                                                                       \
            typedef Type1_ TraitType1_;                                                                             \
            typedef Type2_ TraitType2_;                                                                             \
            static ConversionRegistrator##Type1_##Type2_ traitRegistrator_;                                         \
        };                                                                                                          \
        ConversionRegistrator##Type1_##Type2_ ConversionRegistrator##Type1_##Type2_::traitRegistrator_ = ConversionRegistrator##Type1_##Type2_();    \
    }                                                                                                               \



#endif