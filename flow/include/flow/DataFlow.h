//-----------------------------------------------------------------------------
//  FLOW
//-----------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//-----------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to
//  deal in the Software without restriction, including without limitation the
//  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
//  sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//  IN THE SOFTWARE.
//-----------------------------------------------------------------------------

#ifndef FLOW_DATAFLOW_H_
#define FLOW_DATAFLOW_H_

#include <flow/Export.h>
#include <flow/ThreadPool.h>

#include <boost/any.hpp>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <flow/PolicyInput.h>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace flow {

/// Base class of flow that a flow of data. It manages the implicit conversion
/// of data through the streams and call the associated callbacks. It is used in
/// input policies to generate automatic data flows.
/// @ingroup  flow
class DataFlow {
public:
  /// Creates a new dataflow with a set of tags (including types) and a callback
  /// to be called when the data arrives
  template <typename... Arguments>
  static DataFlow *create(const std::vector<PolicyInput> &_inputs,
                          std::function<void(Arguments... _args)> _callback);

  /// Creates a new dataflow with a set of tags (including types) and a method
  /// and a class instance to be called when the data arrives.
  template <typename T_, typename... Arguments>
  static DataFlow *create(const std::vector<PolicyInput> &_inputs,
                          void (T_::*_callback)(Arguments... _args), T_ *_obj);

  /// Update an specific input tag. If all the inputs have been satisfied then
  /// the callback is called.
  void update(std::string _tag, boost::any _data);
  // 666 OLD INTERFACE Templatized method to get data of any type. Get methods
  // are implemented all around the different plugins template<typename T_> T_
  // get(std::string const &_tag);

private:
  /// Query to check internal status of data and update the DataFlow
  void checkData();

  template <typename T_> static T_ castData(boost::any _data);

  /// Construct a data flow with a set if input flows and associate a callback
  /// to it.
  FLOW_DECL DataFlow(const std::vector<PolicyInput> &_inputs);

  template <typename... Arguments>
  void prepareCallback(const std::vector<std::string> &_listTags,
                       std::function<void(Arguments... _args)> _callback);

private:
  std::map<std::string, std::string> types_;
  std::map<std::string, boost::any> data_;
  std::map<std::string, bool> updated_;
  std::map<std::string, bool> isConsumable_;
  std::function<void(const std::map<std::string, boost::any> &)> callback_;
  std::mutex safeCopyLock_;
  bool isRunning_ = false;
  ThreadPool pool_;

public:
  static FLOW_DECL
      std::map<std::string,
               std::map<std::string, std::function<boost::any(boost::any &)>>>
          conversions_;
  static bool checkIfConversionAvailable(std::string const &_from,
                                         std::string const &_to);
};

} // namespace flow

namespace flow {

template <typename... Arguments>
DataFlow *DataFlow::create(const std::vector<PolicyInput> &_inputs,
                           std::function<void(Arguments... _args)> _callback) {

  std::vector<std::string> listTags;
  for (const auto &input : _inputs) {
    listTags.push_back(input.tag());
  }

  auto *df = new DataFlow(_inputs);
  df->prepareCallback(listTags, _callback);

  return df;
}

template <typename T_, typename... Arguments>
DataFlow *DataFlow::create(const std::vector<PolicyInput> &_inputs,
                           void (T_::*_cb)(Arguments... _args), T_ *_obj) {
  std::function<void(Arguments... _args)> fn = [_cb, _obj](Arguments... _args) {
    (_obj->*_cb)(_args...);
  };

  return create(_inputs, fn);
}

template <typename... Arguments>
void DataFlow::prepareCallback(
    const std::vector<std::string> &_listTags,
    std::function<void(Arguments... _args)> _callback) {

  // We need to create a template lambda to autoexpand the index_sequence to the
  // size of the template parameter Arguments...
  auto tmpCb = [&]<std::size_t... Is>(
      std::map<std::string, boost::any> _data, std::vector<std::string> _tags,
      std::function<void(Arguments... _args)> _insideCb,
      std::index_sequence<Is...> const &) {

    std::vector<boost::any> parsedData;
    for (const auto &t : _tags) {
      if (_data.find(t) != _data.end()) {
        parsedData.push_back(_data[t]);
      } else {
        return; // One of the inputs is not present, do not proceed or will
                // crash
      }
    }

    _insideCb(castData<Arguments>(parsedData[Is])...);
    isRunning_ = false; // Finished running, can release the "resource"
  };

  // Then se save and pack everything inside of a simple functor.
  callback_ = std::bind(tmpCb, std::placeholders::_1, _listTags, _callback,
                        std::make_index_sequence<sizeof...(Arguments)>{});
}

template <typename T_> inline T_ DataFlow::castData(boost::any _data) {
  if (strcmp(typeid(T_).name(), _data.type().name()) == 0) {
    return boost::any_cast<T_>(_data);
  } else {
    if (auto iter = conversions_.find(_data.type().name());
        iter != conversions_.end()) {
      if (iter->second.find(typeid(T_).name()) != iter->second.end()) {
        std::function<boost::any(boost::any &)> fn =
            iter->second[typeid(T_).name()];
        return boost::any_cast<T_>(fn(_data));
      }
    }
    // No data conversion is available... Then try some basic creations
    if constexpr (std::is_arithmetic_v<T_>)
      return 0;
    else if constexpr (std::is_default_constructible_v<T_>)
      return T_();
    else
      throw std::invalid_argument(
          "Bad tag type when getting data from DataFlow");
  }
}

template <> inline boost::any DataFlow::castData(boost::any _data) {
  return _data;
}

// 666 OLD INTERFACE, about to be removed
// template<typename T_>
// inline T_ DataFlow::get(std::string const &_tag){
//    if(types_.find(_tag) != types_.end()){
//        //throw std::invalid_argument("Input tag does not exist, Add it as
//        policy"); if(strcmp(typeid(T_).name(), data_[_tag].type().name()) == 0
//        ){
//            return boost::any_cast<T_>(data_[_tag]);
//        }else{
//            if( auto iter = conversions_.find(data_[_tag].type().name()); iter
//            != conversions_.end()){
//                if(iter->second.find(typeid(T_).name()) !=
//                iter->second.end()){
//                    std::function<boost::any(boost::any&)> fn =
//                    iter->second[typeid(T_).name()]; return
//                    boost::any_cast<T_>(fn(data_[_tag]));
//                }
//            }
//        }
//    }
//

//    if constexpr (std::is_arithmetic_v<T_>)
//        return 0;
//    else if constexpr (std::is_default_constructible_v<T_>)
//        return T_();
//    else
//        throw std::invalid_argument("Bad tag type when getting data from
//        DataFlow");
//}

// template<>
// inline boost::any DataFlow::get(std::string const& _tag) {
//    return data_[_tag];
//}
} // namespace flow
#define FLOW_CONVERSION_REGISTER(Type1_, Type2_, conversion_)                  \
  namespace flow {                                                             \
  struct ConversionRegistrator##Type1_##Type2_ {                               \
    ConversionRegistrator##Type1_##Type2_() {                                  \
      DataFlow::conversions_[typeid(Type1_).name()][typeid(Type2_).name()] =   \
          conversion_;                                                         \
    }                                                                          \
    typedef Type1_ TraitType1_;                                                \
    typedef Type2_ TraitType2_;                                                \
    static ConversionRegistrator##Type1_##Type2_ traitRegistrator_;            \
  };                                                                           \
  ConversionRegistrator##Type1_##Type2_                                        \
      ConversionRegistrator##Type1_##Type2_::traitRegistrator_ =               \
          ConversionRegistrator##Type1_##Type2_();                             \
  }

#endif