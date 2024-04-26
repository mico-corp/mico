//-----------------------------------------------------------------------------
//   fastcom wrapper MICO plugin
//-----------------------------------------------------------------------------
//   Copyright 2020 - Marco Montes Grova (a.k.a. mgrova) marrcogrova@gmail.com
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

#ifndef MICO_FASTCOM_WRAPPER_BLOCK_SUBSCRIBER_H_
#define MICO_FASTCOM_WRAPPER_BLOCK_SUBSCRIBER_H_

#include <fastcom/Subscriber.h>
#include <flow/flow.h>

namespace fastcom_wrapper {
template <typename Trait_> class BlockFastcomSubscriber : public flow::Block {
public:
  BlockFastcomSubscriber() {
    createPipe<typename Trait_::DataType_>(Trait_::outputName());
  };

  /// Retreive icon of block
  std::string icon() const override {
    return (flow::Persistency::resourceDir() + "fastcom/fastcom_logo.png");
  }

  std::string name() const override { return Trait_::blockName(); }

  /// Returns a brief description of the block
  std::string description() const override {
    return "Communication block using fastcom.\n" +
           std::string("Subscriber actor that receive data .");
  };

  /// Configure block with given parameters.
  bool configure(std::vector<flow::ConfigParameterDef> _params) override {
    std::string ipAdress = "127.0.0.1";
    if (auto param = getParamByName(_params, "ip"); param) {
      ipAdress = param.value().asString();
    }

    if (auto param = getParamByName(_params, "uri"); param) {
      traitSub_.sub_ =
          new typename Trait_::Subscriber_(param.value().asString());
    }

    traitSub_.sub_->addCallback([&](const typename Trait_::FastcomType_ &_msg) {
      typename Trait_::DataType_ fMsg(_msg);
      if (getPipe(Trait_::outputName())->registrations() != 0) {
        getPipe(Trait_::outputName())->flush(fMsg);
      }
    });

    return true;
  };

  /// Get list of parameters of the block
  std::vector<flow::ConfigParameterDef> parameters() override {
    return {{"ip", flow::ConfigParameterDef::eParameterType::STRING,
             std::string("127.0.0.1")},
            {"uri", flow::ConfigParameterDef::eParameterType::STRING,
             std::string("/uri")}};
  };

  /// Return if the block is configurable.
  bool isConfigurable() override { return true; };

private:
  Trait_ traitSub_;
  std::function<void(const typename Trait_::FastcomType_ &)> callback_;
};

} // namespace fastcom_wrapper
#endif
