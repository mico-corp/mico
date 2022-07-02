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



#ifndef MICO_FLOW_BLOCKS_STREAMERS_BLOCKDFT_H_
#define MICO_FLOW_BLOCKS_STREAMERS_BLOCKDFT_H_

#include <flow/Block.h>
#include <vector>


namespace mico{
    namespace audio{
        /// The block takes a vector as input and computes its DFT
        ///
        /// __Outputs__:
        ///
        /// __parameters__:
        ///
        ///
        class BlockDFT:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "DFT";}     
            
            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir() / "audio" / "icon_fft.svg").string();
            }
            
            /// Base constructor
            BlockDFT();

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override {return    "The block takes a vector as input and computes its DFT.\n"
                                                                "   - Inputs: \n";};
        private:
            void computeDFT(std::vector<float> _signal);

        private:
            std::vector<float> bufferData_;
            float silenceThreshold_ = 0.02f;
            int silenceSamples_ = 100;
            int samplesCounter_ = 0;
            bool isRecording_ = false;
        };
    }
}



#endif