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



#ifndef MICO_FLOW_BLOCKS_STREAMERS_BlockSpectrogram_H_
#define MICO_FLOW_BLOCKS_STREAMERS_BlockSpectrogram_H_

#include <flow/Block.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <fftw3.h>

namespace mico{
    namespace audio{
        /// Computes the espectrogram of an stream of audio
        ///
        /// __Outputs__:
        ///
        /// __parameters__:
        ///
        ///
        class BlockSpectrogram:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Spectrogram";}     
            
            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir() / "audio" / "spectrogram.svg").string();
            }
            
            /// Base constructor
            BlockSpectrogram();

            /// Base destructor
            ~BlockSpectrogram();

            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            std::vector<flow::ConfigParameterDef> parameters() override;
            
            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Computes the espectrogram of an stream of audio.\n"
                                                                "   - Inputs: \n";};
        private:
            void inputCallback(std::vector<float> _input);
            void computeDFT(const std::vector<float>& _input);

        private:
            std::vector<float> bufferData_;
            std::mutex bufferLock_;
            fftw_complex* rawDft_ = nullptr;

            cv::Mat spectrogram_;
            int sampleSize_ = 2048;
            int stepSize_ = 512;
            int sizeSpectrogram_ = 100;
            int maxFrequency_ = 100;
            int currentSample_ = 0;
        
        };
    }
}



#endif