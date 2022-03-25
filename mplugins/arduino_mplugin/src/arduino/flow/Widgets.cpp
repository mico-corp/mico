//---------------------------------------------------------------------------------------------------------------------
//  Arduino MICO plugin
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


#include <mico/arduino/flow/Widgets.h>

#include <QVBoxLayout>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QGroupBox>
#include <QPixmap>

namespace mico{
    namespace arduino {
        ToggleButtonBlock::ToggleButtonBlock(){
            createPipe<bool>("state");

            button_ = new QPushButton("Toggle me");
            button_->setCheckable(true);
            QObject::connect(button_, &QPushButton::clicked,[&](){
                getPipe("state")->flush((bool)button_->isChecked());
            });
        }

        QWidget * ToggleButtonBlock::customWidget(){
            return button_;
        }

        
        SliderPwm::SliderPwm(){
            createPipe<int>("pwm");

            slider_ = new QSlider();
            slider_->setMinimum(0);
            slider_->setMaximum(255);
            
            QObject::connect(slider_, &QSlider::valueChanged,[&](int _val){
                getPipe("pwm")->flush(_val);
            });
        }

        QWidget * SliderPwm::customWidget(){
            return slider_;
        }

        SignalSwitcher::SignalSwitcher() {
            
            customWidget_ = new QGroupBox();

            QVBoxLayout* l = new QVBoxLayout();
            customWidget_->setLayout(l);

            img_ = new QLabel();
            img_->setMaximumHeight(50);
            img_->setMaximumWidth(50);
            img_->setPixmap(QIcon(fileA.c_str()).pixmap(50,50));
            l->addWidget(img_);

            createPipe<boost::any>("Out");
            createPolicy({  flow::makeInput<bool>("Signal"),
                            flow::makeInput<boost::any>("A"),
                            flow::makeInput<boost::any>("B")});

            registerCallback({ "Signal" },
                [&](flow::DataFlow _data) {
                    flowA_ = _data.get<bool>("Signal");
                    if (flowA_) {
                        img_->setPixmap(QIcon(fileA.c_str()).pixmap(50, 50));
                    } else {
                        img_->setPixmap(QIcon(fileB.c_str()).pixmap(50, 50));
                    }
                }
            );

            registerCallback({ "A" },
                [&](flow::DataFlow _data) {
                    if (auto pipe = getPipe("Out"); pipe->registrations() && flowA_)
                        pipe->flush(_data.get<boost::any>("A"));
                }
            );

            registerCallback({ "B" },
                [&](flow::DataFlow _data) {
                    if (auto pipe = getPipe("Out"); pipe->registrations() && !flowA_)
                        pipe->flush(_data.get<boost::any>("B"));
                }
            );
        }

        QWidget* SignalSwitcher::customWidget() {
            return customWidget_;
        }
    }
}