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

#ifndef MICO_FLOW_BLOCKS_STREAMERS_STREAMREALSENSE_TRACKING_H_
#define MICO_FLOW_BLOCKS_STREAMERS_STREAMREALSENSE_TRACKING_H_

#ifdef ENABLE_LIBREALSENSE_V2

#include <mico/cameras/TrackingCamera.h>

#include <flow/Block.h>
#include <flow/Outpipe.h>

#include <QGroupBox>
#include <QPushButton>

namespace mico{

    class StreamRealSenseTracking : public flow::Block{
    public:
        /// Get name of block
        virtual std::string name() const override {return "RealSense T265 Streamer";}     
        /// Retreive icon of block    
            virtual QIcon icon() const override { 
            std::string userDir(getenv("USER"));
            std::string resourcesDir = "/home/"+userDir+"/.flow/plugins/resources/cameras/";
            return QIcon((resourcesDir+"realsense_icon.png").c_str());
        }
        
        StreamRealSenseTracking();
        ~StreamRealSenseTracking(){};
        
        /// Configure block with given parameters.
        virtual bool configure(std::vector<flow::ConfigParameterDef> _params) override;
        /// Get list of parameters of the block
        std::vector<flow::ConfigParameterDef> parameters() override;
    
        /// Returns a brief description of the block
        std::string description() const override {return    "Streamer block that reads from an Intel realsense device and streams its flows of images and poses.\n"
                                                            "   - Outputs: \n";};

        virtual /// Get custom view widget to be display in the graph
        QWidget * customWidget() override;
    protected:
        virtual void loopCallback() override;

    private:
        TrackingCamera camera_;
        bool hasInitCamera_ = false;
    };

}

#endif

#endif