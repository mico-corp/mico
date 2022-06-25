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



#ifndef MICO_FLOW_BLOCKS_STREAMERS_SPEAKERSBLOCK_H_
#define MICO_FLOW_BLOCKS_STREAMERS_SPEAKERSBLOCK_H_

#include <flow/Block.h>
#include <audio>
#include <vector>


namespace mico{
    namespace audio{
        /// The block receives data from the input and puts it into the buffer to be run by the audio device
        ///
        /// __Outputs__:
        ///
        /// __parameters__:
        ///
        ///
        class SpeakersBlock:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Speakers";}     
            
            /// Retreive icon of block    
            QIcon icon() const override { 
                return QIcon((flow::Persistency::resourceDir() / "audio" / "icon_speaker.png").string().c_str());
            }
            
            /// Base constructor
            SpeakersBlock();

            /// Base destructor
            ~SpeakersBlock();

            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "The block receives data from the input and puts it into the buffer to be run by the audio device.\n"
                                                                "   - Inputs: \n";};
        private:
            void fillBuffer(const std::vector<float>& _input, unsigned _channel);
            void audioCallback(std::experimental::audio_device&, std::experimental::audio_device_io<float>& io);

        private:
            std::optional<std::experimental::audio_device> device_;
            unsigned nChannels_;
            std::vector<std::vector<float>> bufferData_;
            std::mutex dataLock_;
        };
    }
}



#endif