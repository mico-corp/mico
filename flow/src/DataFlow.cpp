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

#include <flow/DataFlow.h>
#include <flow/ThreadPool.h>
#include <thread>

namespace flow {

std::map<std::string,
         std::map<std::string, std::function<boost::any(boost::any &)>>>
    DataFlow::conversions_ = {};

void DataFlow::update(std::string _tag, boost::any _data) {
  if (data_.find(_tag) != data_.end()) {
    // Can we check here the type?
    safeCopyLock_.lock();
    data_[_tag] = _data;
    safeCopyLock_.unlock();
    updated_[_tag] = true;
    checkData();
  }
}

void DataFlow::checkData() {
  if (isRunning_)
    return; // Don't even try to run it if it is busy.
  isRunning_ = true;

  size_t flagCounter = 0;
  for (auto flag = updated_.begin(); flag != updated_.end(); flag++) {
    if (flag->second)
      flagCounter++;
  }
  if (flagCounter == updated_.size()) {
    // Emplace task
    safeCopyLock_.lock();
    ThreadPool::get()->emplace(std::bind(callback_, data_));
    safeCopyLock_.unlock();

    // Consume consumable data
    for (const auto &[tag, isConsumable] : isConsumable_) {
      updated_[tag] = !isConsumable;
    }
  } else {
    isRunning_ = false;
  }
}

bool DataFlow::checkIfConversionAvailable(std::string const &_from,
                                          std::string const &_to) {
  if (_from == _to)
    return true;

  if (conversions_.find(_from) != conversions_.end()) {
    if (conversions_[_from].find(_to) != conversions_[_from].end()) {
      return true;
    }
  }

  if (strcmp(typeid(boost::any).name(), _to.c_str()) == 0 ||
      strcmp(typeid(boost::any).name(), _from.c_str()) == 0) {
    return true;
  }

  return false;
}

DataFlow::DataFlow(const std::vector<PolicyInput> &_inputs) {
  for (auto &input : _inputs) {
    types_[input.tag()] = input.typeName();
    data_[input.tag()] = boost::any();
    updated_[input.tag()] = false;
    isConsumable_[input.tag()] = input.isConsumable();
  }
}

} // namespace flow
