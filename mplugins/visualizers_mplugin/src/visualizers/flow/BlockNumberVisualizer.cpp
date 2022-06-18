//---------------------------------------------------------------------------------------------------------------------
//  Visualizers MICO plugin
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



#include <mico/visualizers/flow/BlockNumberVisualizer.h>

#include <flow/Policy.h>

#include <QLabel>
#include <QTimer>

namespace mico{
    namespace visualizer{
        BlockNumberVisualizer::BlockNumberVisualizer(){
            textDisplay_ = new QLabel("0.000000");

            createPolicy({  flow::makeInput<float>("Number")});

            registerCallback<float>({"Number"}, 
                                    [&](float _number){
                                        number_ = _number;
                                        // textDisplay_->setText(std::to_string(number_).c_str());
                                    }
                                );

            refreshTimer_ = new QTimer();
            QObject::connect(refreshTimer_, &QTimer::timeout, [&](){
                textDisplay_->setText(std::to_string(number_).c_str());
            });
            refreshTimer_->start(30);
        }


        QWidget * BlockNumberVisualizer::customWidget(){
            return textDisplay_;
        }

        BlockNumberVisualizer::~BlockNumberVisualizer(){
            refreshTimer_->stop();
        }
    }
}
