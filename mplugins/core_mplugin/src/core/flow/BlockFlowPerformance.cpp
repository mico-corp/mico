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


#include <mico/core/flow/BlockFlowPerformance.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>
#include <flow/ThreadPool.h>

#include <cmath>
#include <sstream>

#include <QLabel>
#include <QTimer>

namespace mico{
    namespace core{
        BlockFlowPerformance::BlockFlowPerformance(){
            textDisplay_ = new QLabel("0.000000");
            refreshTimer_ = new QTimer();
            QObject::connect(refreshTimer_, &QTimer::timeout, [&]() {
                textDisplay_->setText(std::to_string(flow::ThreadPool::get()->loadRatio()).c_str());
            });
            refreshTimer_->start(100);
        }

        BlockFlowPerformance::~BlockFlowPerformance() {
            refreshTimer_->stop();
        }


        QWidget* BlockFlowPerformance::customWidget() {
            return textDisplay_;
        }

    }
}