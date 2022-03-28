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

#ifndef DVSAL_PROCESSORS_CORNERDETECTORS_HARRIS_DETECTOR_H_
#define DVSAL_PROCESSORS_CORNERDETECTORS_HARRIS_DETECTOR_H_

#include <deque>

#include "dvsal/processors/corner_detectors/Detector.h"

#include "dvsal/processors/corner_detectors/utils/LocalEventQueues.h"
#include "dvsal/processors/corner_detectors/utils/DistinctQueue.h"

namespace dvsal{

  class HarrisDetector : public Detector{
  public:
    HarrisDetector();
    virtual ~HarrisDetector();

    bool isFeature(const dv::Event &e);

    virtual std::string name() override {return "HARRIS";}
    virtual QWidget * customWidget() override {return nullptr;}

    double getLastScore() const {
      return lastScore_;
    }

  private:
    // methods
    void updateQueue(const int x, const int y, const dv::Event &e);
    double getHarrisScore(int x, int y, bool polarity);

    // queues
    LocalEventQueues* queues_;

    // parameters
    int queueSize_;
    int windowSize_;
    int kernelSize_;
    static const int sensorWidth_  = 240;
    static const int sensorHeight_ = 180;
    double harrisThreshold_;

    double lastScore_;

    // kernels
    Eigen::MatrixXd Gx_, h_;
    int factorial(int n) const;
    int pasc(int k, int n) const;
  };


} // namespace

#endif