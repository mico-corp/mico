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

#ifndef FLOW_BLOCK_H_
#define FLOW_BLOCK_H_

#include <flow/Export.h>
#include <flow/Persistency.h>

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/any.hpp>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <boost/any.hpp>
#include <tuple>

#include <QBoxLayout>
#include <QIcon>
#include <QtCore/QObject>
#include <cassert>
#include <optional>

namespace flow {

struct FLOW_DECL ConfigParameterDef {
  enum class eParameterType {
    BOOLEAN,
    INTEGER,
    DECIMAL,
    STRING,
    PATH,
    OPTIONS
  };
  std::string name_;
  eParameterType type_;
  boost::any value_;
  std::string selectedOption_ = "";
  bool asBool() const;
  int asInteger() const;
  float asDecimal() const;
  std::string asString() const;
  fs::path asPath() const;
  std::vector<std::string> asOptions() const;
  std::string selectedOption() const;

  std::string serialize();
};

/// Base class of flow that represents a functioning block. A Block represents
/// an action or module with an specific behaviour or implemented algorithm. Is
/// composed by a set of N inputs and M outputs and the associated internal
/// actions.
/// @ingroup  flow
class FLOW_DECL Block {
public:
  /// Get name of block
  virtual std::string name() const { return "Unnammed"; }

  /// Base destructor
  ~Block();

  // BASE METHODS
  /// Configure block with given parameters.
  virtual bool configure(std::vector<flow::ConfigParameterDef>) {
    return false;
  };

  /// Get list of parameters of the block
  virtual std::vector<flow::ConfigParameterDef> parameters() { return {}; };

  /// Return if the block is configurable. This method is abstract to ensure new
  /// implementations implement it
  virtual bool isConfigurable() = 0;

  [[deprecated("This function gives the map with all pipes, please use getPipe "
               "method and get just the needed")]] std::
      unordered_map<std::string, std::shared_ptr<Outpipe>>
      getPipes();

  /// Get pointer to output pipe by tag
  std::shared_ptr<Outpipe> getPipe(std::string _tag);

  /// If the block is auto runnable, start it.
  void start();

  /// If the block is auto runnable, stop it.
  void stop();

  // void operator()(std::unordered_map<std::string,boost::any> _data,
  // std::unordered_map<std::string,bool> _valid);

  /// Retreive number of inputs.
  size_t nInputs();

  /// Retreive list of input tags
  std::vector<std::string> inputTags();

  /// Retreive number of outputs
  size_t nOutputs();

  /// Retreive list of output tags
  std::vector<std::string> outputTags();

  /// Retreive input policy
  Policy *getPolicy();

  /// Connect an output pipe of current block with an input policy tag of
  /// another block. This method is used by the system for the low level
  /// connection of blocks.
  void connect(std::string _pipeTag, std::string _policyTag,
               Block &_otherBlock);

  /// Disconnect a connected output pipe
  void disconnect(std::string _pipeTag);

  /// Virtual method to be overrided by sons. Get custom view widget to be
  /// display in the graph
  virtual QWidget *customWidget() { return nullptr; };

  /// Virtual method to be overrided by sons. Get custom view widget to be
  /// display in the creation context menu
  virtual QBoxLayout *creationWidget() { return nullptr; };

  /// Virtual method to tell the interface if the visual block is resizable or
  /// not.
  virtual bool resizable() const { return false; };

  /// Method to check if the block has auto running callback
  virtual bool hasRunLoop() const { return false; };

  /// Returns a brief description of the block
  virtual std::string description() const {
    return "Flow block without description";
  };

  /// Virtual method. Retrieve path to the icon of the block. By default empty.
  virtual std::string icon() const { return ""; };

  /// Get specific parameter from list of parameter by name.
  std::optional<ConfigParameterDef>
  getParamByName(const std::vector<flow::ConfigParameterDef> &_params,
                 const std::string &_pname);

protected:
  bool isRunningLoop() const;

  template <typename T_> bool createPipe(std::string _pipeTag);
  bool removePipe(std::string _pipeTag);
  bool removePipes();

  bool createPolicy(std::vector<PolicyInput> _inputs);
  void removePolicy();

  template <typename... Argumens>
  bool registerCallback(Policy::PolicyMask _mask,
                        std::function<void(Argumens... _args)> _callback);

  template <typename T_, typename... Argumens>
  bool registerCallback(Policy::PolicyMask _mask,
                        void (T_::*_cb)(Argumens... _args), T_ *_obj);

protected:
  virtual void loopCallback(){};

private:
  Policy *iPolicy_ = nullptr;
  std::unordered_map<std::string, std::shared_ptr<Outpipe>> opipes_;
  std::thread loopThread_;
  bool runLoop_ = false;
};

template <typename T_> bool Block::createPipe(std::string _pipeTag) {
  if (opipes_[_pipeTag] == nullptr) {
    opipes_[_pipeTag] = std::shared_ptr<Outpipe>(
        new flow::Outpipe(_pipeTag, typeid(T_).name()));
    return true;
  } else {
    throw std::invalid_argument("Pipe with tag " + _pipeTag +
                                " already defined.");
    return false;
  }
}

template <typename... Argumens>
bool Block::registerCallback(Policy::PolicyMask _mask,
                             std::function<void(Argumens... _args)> _callback) {
  return iPolicy_ && iPolicy_->registerCallback(_mask, _callback);
}

template <typename T_, typename... Argumens>
bool Block::registerCallback(Policy::PolicyMask _mask,
                             void (T_::*_cb)(Argumens... _args), T_ *_obj) {
  return iPolicy_ && iPolicy_->registerCallback(_mask, _cb, _obj);
}

} // namespace flow

#endif
