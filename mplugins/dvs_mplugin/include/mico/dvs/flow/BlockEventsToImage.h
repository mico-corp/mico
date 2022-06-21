//---------------------------------------------------------------------------------------------------------------------
//  DVSAL
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 - Marco Montes Grova (a.k.a. mgrova) marrcogrova@gmail.com 
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

#ifndef BLOCK_DVS_IMAGE_VISUALIZER_H_
#define BLOCK_DVS_IMAGE_VISUALIZER_H_

#include <flow/flow.h>
#include <opencv2/opencv.hpp>
#include <mico/dvs/Common.h>

#include <dv-sdk/processing.hpp>
#include <dv-sdk/config.hpp>
#include <dv-sdk/utils.h>

namespace mico{

namespace dvs{

    class BlockEventsToImage: public flow::Block{
    public:
        std::string name() const override {return "DVS events to image";};
        std::string description() const override {return "Flow conversor";};

        BlockEventsToImage();
        
        /// Configure block with given parameters.
        bool configure(std::vector<flow::ConfigParameterDef> _params) override;

        /// Get list of parameters of the block
        std::vector<flow::ConfigParameterDef> parameters() override;

        /// Return if the block is configurable.
        bool isConfigurable() override { return true; };

    private:
        bool obtainImageFromEvents(PolarityPacket _events , cv::Mat &_image);  

    private:

        int imageWidth_  = 128;
        int imageHeight_ = 128;
    };   
}

}
#endif