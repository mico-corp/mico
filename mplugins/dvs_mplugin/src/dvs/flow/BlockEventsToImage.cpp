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

#include <mico/dvs/flow/BlockEventsToImage.h>

#include <mico/dvs/Common.h>

namespace mico{

namespace dvs{

    BlockEventsToImage::BlockEventsToImage(){

        createPipe<cv::Mat>("image");
        
        createPolicy({flow::makeInput<PolarityPacket>("events")});
        registerCallback({"events"},
         &BlockEventsToImage::integrateEvents, this
        );
    }

    void BlockEventsToImage::integrateEvents(PolarityPacket _events){
	    cv::Mat image = cv::Mat::zeros(cv::Size(imageWidth_,imageHeight_), CV_8UC3);
	    if(obtainImageFromEvents(_events,image))
		if(getPipe("image")->registrations() !=0 )
		    getPipe("image")->flush(image); 
	}
        
    bool BlockEventsToImage::configure(std::vector<flow::ConfigParameterDef> _params) {
        if (auto param = getParamByName(_params, "image_width"); param)
            imageWidth_ = param.value().asInteger();

        if (auto param = getParamByName(_params, "image_height"); param)
            imageHeight_ = param.value().asInteger();

        return true;
    }

    std::vector<flow::ConfigParameterDef> BlockEventsToImage::parameters(){
        return {
            {"image_width", flow::ConfigParameterDef::eParameterType::INTEGER, 128},
            {"image_height", flow::ConfigParameterDef::eParameterType::INTEGER, 128}
        };
    }

    bool BlockEventsToImage::obtainImageFromEvents(PolarityPacket _events , cv::Mat &_image){
        for (libcaer::events::EventPacket::size_type i = 0; i < _events->size(); i++) {
            const libcaer::events::PolarityEvent &event = (*_events)[i];
            if(!event.isValid())
                continue;

            const uint16_t x = event.getX();
            const uint16_t y = event.getY();
            const bool pol   = event.getPolarity();
            
            if(y < imageHeight_ &&  x < imageWidth_){
                if (pol)
                    _image.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,255);
                else
                    _image.at<cv::Vec3b>(y,x) = cv::Vec3b(0,255,0);
            }
        }
        return true;
    }  
}
}
