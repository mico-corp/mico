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

#ifdef ENABLE_LIBREALSENSE_V2

#ifndef MICO_FLOW_BLOCKS_STREAMERS_STREAMREALSENSE_H_
#define MICO_FLOW_BLOCKS_STREAMERS_STREAMREALSENSE_H_

#include <flow/Block.h>

#include <mico/cameras/StereoCameras/StereoCameraRealSense.h>
#include <flow/Persistency.h>

namespace mico{

    class StreamRealSense:public flow::Block{
    public:
        /// Get name of block
        virtual std::string name() const override {return "RealSense Streamer";}     
        /// Retreive icon of block    
            virtual QIcon icon() const override { 
            std::string userDir(getenv("USER"));
            std::string resourceDir = flow::Persistency::resourceDir()/"resources"/"cameras"/"realsense_icon.png";
            return QIcon(resourceDir.c_str());
        }
        
        StreamRealSense();
        ~StreamRealSense(){};
        
        /// Configure block with given parameters.
        virtual bool configure(std::vector<flow::ConfigParameterDef> _params) override;
        /// Get list of parameters of the block
        std::vector<flow::ConfigParameterDef> parameters() override;
    
        /// Returns a brief description of the block
        std::string description() const override {return    "Streamer block that reads from an Intel realsense device and streams its flows of images.\n"
                                                            "   - Outputs: \n";};
    protected:
        virtual void loopCallback() override;

    private:
        StereoCameraRealSense camera_;
        bool hasInitCamera_ = false;
    };

}



#endif

#endif