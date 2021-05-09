//---------------------------------------------------------------------------------------------------------------------
//  Cameras wrapper MICO plugin
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



#ifndef MICO_FLOW_BLOCKS_FACEDETECTORS_H_
#define MICO_FLOW_BLOCKS_FACEDETECTORS_H_

#include <flow/Block.h>

#include <opencv2/opencv.hpp>

namespace mico{
    namespace ml{
        /// Mico block that detects different objects using a Haar Cascade classifier.
        /// @ingroup  mico_ml
        class BlockHaarCascade:public flow::Block{
        public:
            /// Get name of block
            virtual std::string name() const override {return "Haar Cascade Clasifier";}        

            /// Base constructor. Initializes the classifier.
            BlockHaarCascade();

            /// Configure block with given parameters.
            virtual bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Detect objects Haars cascade algorithm"
                                                                "   - Inputs: \n"
                                                                "   - Outputs: \n";};

        private:
            cv::CascadeClassifier face_cascade;
            bool idle_ = true;
            bool isConfigured_ = false;

            std::map<std::string, std::string> detectors = {
                {"Face detector",  (flow::Persistency::resourceDir() / "ml"/"haarcascade_frontalface_alt.xml").string()},
                {"Body detector",  (flow::Persistency::resourceDir() / "ml"/"haarcascade_fullbody.xml").string()},
                {"Upperbody detector",  (flow::Persistency::resourceDir() / "ml"/"haarcascade_upperbody.xml").string()}
            };
        };

    }
}



#endif