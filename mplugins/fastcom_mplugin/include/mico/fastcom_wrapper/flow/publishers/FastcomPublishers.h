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

#ifndef MICO_FASTCOM_WRAPPER_PUBLISHERS_H_
#define MICO_FASTCOM_WRAPPER_PUBLISHERS_H_

#include <flow/flow.h>
#include <mico/fastcom_wrapper/flow/SerializableVector.h>
#include <mico/fastcom_wrapper/flow/SerializableMat.h>
#include <mico/fastcom_wrapper/flow/publishers/BlockFastcomPublisher.h>

namespace fastcom_wrapper{

    //-----------------------------------------------------------------------------------------------------------------
    template <typename FlowType_, typename FastcomType_, char const *Name_, char const *InputName_, char const *InputType_>
    struct TraitFastcomPublisher{
        typedef FlowType_ DataType_;
        typedef fastcom::Publisher<FastcomType_> Publisher_;

        static std::string blockName(){ return Name_; };
        static std::string inputName(){ return InputName_; };
        static std::string inputType(){ return InputType_; };

        auto dataCallback(flow::DataFlow _data) {
            if(pub_!= nullptr){
                auto flowData = _data.get<FlowType_>(inputName());
                auto fastcomData(flowData);
                pub_->publish(fastcomData);
            }
        }
        Publisher_ *pub_;

    };

    //-----------------------------------------------------------------------------------------------------------------
    char const ifpn[] = "Fastcom Publisher int";
    char const ifpin[] = "integer";
    char const ifpit[] = "int";
    typedef TraitFastcomPublisher<  int,
                                    int,
                                    ifpn,
                                    ifpin,
                                    ifpit> TraitFastcomIntPublisher;
                                    
    typedef BlockFastcomPublisher< TraitFastcomIntPublisher     > BlockFastcomIntPublisher;


    //-----------------------------------------------------------------------------------------------------------------  
    char const ffpn[] = "Fastcom Publisher float";
    char const ffpin[] = "float";
    char const ffpit[] = "float";  
    typedef TraitFastcomPublisher<  float,
                                    float,
                                    ffpn,
                                    ffpin,
                                    ffpit> TraitFastcomFloatPublisher;

    typedef BlockFastcomPublisher< TraitFastcomFloatPublisher   > BlockFastcomFloatPublisher;
    
    //-----------------------------------------------------------------------------------------------------------------
    char const imgfpn[] = "Fastcom Publisher Image";
    char const imgfpin[] = "Image";
    char const imgfpit[] = "image";
    typedef TraitFastcomPublisher<  cv::Mat,
                                    SerializableMat,
                                    imgfpn,
                                    imgfpin,
                                    imgfpit> TraitFastcomImagePublisher;



    typedef BlockFastcomPublisher< TraitFastcomImagePublisher     > BlockFastcomImagePublisher;

    // typedef TraitFastcomPublisher< fastcom::Publisher<std::vector<float>> , std::vector<float>  > TraitFastcomVectorfPublisher;
    // typedef BlockFastcomPublisher< TraitFastcomVectorfPublisher > BlockFastcomVectorfPublisher;

}

#endif