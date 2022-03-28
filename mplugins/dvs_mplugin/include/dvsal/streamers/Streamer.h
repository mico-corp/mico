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

#ifndef DVS_STREAMER_H_
#define DVS_STREAMER_H_

#include <string>
#include <iostream>

#include <dv-sdk/processing.hpp>
#include <dv-sdk/config.hpp>
#include <dv-sdk/utils.h>

#include <opencv2/opencv.hpp>

namespace dvsal{

  class Streamer{
  public:
    template<typename _Type, typename... _T>
    static Streamer *create( _T&&... _arg);
    
    virtual ~Streamer() {};

  public:
    
    virtual bool init() = 0;
    virtual bool step() = 0;
    
    virtual void events(dv::EventStore &_events , int _microseconds = 0) = 0;
    virtual bool image(cv::Mat &_image) = 0; // Fake image using events

    virtual dv::EventStore lastEvents() = 0;

  };    
}

#include "Streamer.inl"

#endif