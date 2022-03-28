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

#ifndef CAMERA_DVS128_STREAMER_H_
#define CAMERA_DVS128_STREAMER_H_

// #define LIBCAER_FRAMECPP_OPENCV_INSTALLED 0
#include <libcaercpp/devices/dvs128.hpp>

#include <atomic>
#include <csignal>

#include <dvsal/streamers/Streamer.h>

namespace dvsal{

    class CameraDVS128Streamer : public Streamer{
    public:
        CameraDVS128Streamer(){};
        ~CameraDVS128Streamer(){};

		bool init();
		void events(dv::EventStore &_events , int _microseconds);
        bool image(cv::Mat &_image); // Fake image using events
        bool step();

        dv::EventStore lastEvents(){
            return lastEvents_;
        };

    private:
        static void usbShutdownHandler(void *_ptr) ;
    private:
        libcaer::devices::dvs128 *dvs128Handle_ = nullptr;        
        constexpr static std::atomic<bool> globalShutdown_{false};

        dv::EventStore lastEvents_;
    };
}

#endif