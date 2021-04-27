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

#ifndef MICO_FASTCOM_WRAPPER_SUBSCRIBERS_H_
#define MICO_FASTCOM_WRAPPER_SUBSCRIBERS_H_

#include <flow/flow.h>

#include <mico/fastcom_wrapper/flow/SerializableVector.h>
#include <mico/fastcom_wrapper/flow/SerializableMat.h>
#include <mico/fastcom_wrapper/flow/streamers/BlockFastcomSubscriber.h>

namespace fastcom_wrapper{
	
    //-----------------------------------------------------------------------------------------------------------------
    template <typename FlowType_, typename FastcomT_, char const *Name_, char const *OutputName_, char const *OutputType_>
    struct TraitFastcomSubscriber{
        typedef FlowType_ DataType_;
        typedef FastcomT_ FastcomType_;
	    typedef fastcom::Subscriber<FastcomType_> Subscriber_;

        static std::string blockName(){ return Name_; };
        static std::string outputName(){ return OutputName_; };
        static std::string outputType(){ return OutputType_; };

        Subscriber_ *sub_;
    };

    //-----------------------------------------------------------------------------------------------------------------
    char const ifsn[] = "Fastcom Subscriber int";
    char const ifson[] = "integer";
    char const ifsot[] = "int";
    typedef TraitFastcomSubscriber<     int, 
                                        int,
                                        ifsn,
                                        ifson,
                                        ifsot> TraitFastcomIntSubscriber;
    
    typedef BlockFastcomSubscriber< TraitFastcomIntSubscriber     > BlockFastcomIntSubscriber;

    //-----------------------------------------------------------------------------------------------------------------
    char const ffsn[] = "Fastcom Subscriber float";
    char const ffson[] = "float";
    char const ffsot[] = "float";   
    typedef TraitFastcomSubscriber<     float              , 
                                        float,
                                        ffsn,
                                        ffson,
                                        ffsot > TraitFastcomFloatSubscriber;
    

    typedef BlockFastcomSubscriber< TraitFastcomFloatSubscriber   > BlockFastcomFloatSubscriber;

    //-----------------------------------------------------------------------------------------------------------------
    char const imagefsn[] = "Fastcom Subscriber image";
    char const imagefson[] = "image";
    char const imagefsot[] = "image";   
    typedef TraitFastcomSubscriber<     cv::Mat              , 
                                        SerializableMat,
                                        imagefsn,
                                        imagefson,
                                        imagefsot > TraitFastcomImageSubscriber;
    

    typedef BlockFastcomSubscriber< TraitFastcomImageSubscriber   > BlockFastcomImageSubscriber;


    // typedef TraitFastcomSubscriber< fastcom::Subscriber<std::vector<float>> , std::vector<float>  > TraitFastcomVectorfSubscriber;
    // typedef BlockFastcomSubscriber< TraitFastcomVectorfSubscriber > BlockFastcomVectorfSubscriber;


}

#endif
