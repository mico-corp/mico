//---------------------------------------------------------------------------------------------------------------------
//   fastcom wrapper MICO plugin
//---------------------------------------------------------------------------------------------------------------------
//   Copyright 2020 - Marco Montes Grova (a.k.a. mgrova) marrcogrova@gmail.com 
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

#ifndef MICO_FASTCOM_WRAPPER_BLOCK_PUBLISHER_H_
#define MICO_FASTCOM_WRAPPER_BLOCK_PUBLISHER_H_


#include <fastcom/Publisher.h>
#include <flow/flow.h>

namespace fastcom_wrapper{
    template<typename Trait_ >
    class BlockFastcomPublisher : public flow::Block{
    public:
		BlockFastcomPublisher(){
            createPolicy({ flow::makeInput<typename Trait_::DataType_>(Trait_::inputName()) });
            
            registerCallback({Trait_::inputName()}, [&](flow::DataFlow _data){
                tPub_.dataCallback(_data);
            } );       
        };

        /// Retreive icon of block    
        std::string icon() const override {
            return (flow::Persistency::resourceDir() + "fastcom/fastcom_logo.png");
        }

        std::string name() const override{return Trait_::blockName(); }

        /// Returns a brief description of the block
        std::string description() const override {return    "Communication block using fastcom.\n"+
                                                            std::string("Publisher actor that sends data .");};

        /// Configure block with given parameters.
        bool configure(std::vector<flow::ConfigParameterDef> _params) override{
            if (auto param = getParamByName(_params, "uri"); param) {
                tPub_.pub_ = new typename Trait_::Publisher_(param.value().asString());
            }
            return true;
        };

        /// Get list of parameters of the block
        std::vector<flow::ConfigParameterDef> parameters() override{
            return {
			    {"uri", flow::ConfigParameterDef::eParameterType::STRING, std::string("/uri")}
            };
        };

        /// Return if the block is configurable.
        bool isConfigurable() override { return true; };
        
    private:
        Trait_ tPub_; // 666
        
    };


    // template<>
    // template<typename Trait_>
    // inline void BlockFastcomPublisher<Trait_>::dataCallback<cv::Mat>(flow::DataFlow _data){
    //     if(idle_){
    //         idle_ = false;
    //         auto data = _data.get<typename Trait_::DataType_>(Trait_::inputName());
    //         SerializableMat smat(data, 100);
    //         pub_->publish(smat);
    //         idle_ = true;  
    //     }
    // }

}

#endif
