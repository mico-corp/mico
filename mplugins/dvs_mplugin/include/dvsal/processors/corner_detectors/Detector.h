//---------------------------------------------------------------------------------------------------------------------
//  CORNER DETECTOR https://github.com/uzh-rpg/rpg_corner_events
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018
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

#ifndef DVSAL_PROCESSORS_CORNERDETECTORS_DETECTOR_H_
#define DVSAL_PROCESSORS_CORNERDETECTORS_DETECTOR_H_

#include <dv-sdk/processing.hpp>
#include <dv-sdk/config.hpp>
#include <dv-sdk/utils.h>

#include <QComboBox>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QLineEdit>

#include <iostream>
namespace dvsal{

  class Detector{
    public:
      Detector();
      virtual ~Detector();

      // check if event
      virtual bool isFeature(const dv::Event &e) = 0;

      virtual dv::EventStore cornersDetected(){
        return cornersDetected_;
      }

      // interface
      void eventCallback(const dv::EventStore &_msg);

      virtual QWidget * customWidget() = 0;
      virtual std::string name() = 0;

    protected:
      std::string detectorName_;

    private:
      dv::EventStore cornersDetected_;
      
  };
}

#endif
