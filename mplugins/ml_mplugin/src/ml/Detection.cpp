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

#ifndef MICO_ML_DETECTION_H_
#define MICO_ML_DETECTION_H_

#include <mico/ml/Detection.h>

#include <QComboBox>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QSpinBox>

namespace mico{
    namespace ml{
        BlockTriggerDetection::BlockTriggerDetection(){
            
            labelSelection_ = new QSpinBox();
            typeSelection_ = new QComboBox();
            typeSelection_->addItem("Has label");
            typeSelection_->addItem("Has not label");

            createPipe<bool>("trigger");
            createPolicy({  flow::makeInput<std::vector<Detection>>("detections") });

            registerCallback(   {"detections"}, 
                                [&](flow::DataFlow _data){
                                    if(!idle_)
                                        return;

                                    idle_ = false;
                                    if(getPipe("trigger")->registrations()){
                                        auto detections = _data.get<std::vector<Detection>>("detections"); 
                                        auto result = std::find_if(detections.begin(), detections.end(), [&](const Detection &_d){
                                            return _d.label_ == labelSelection_->value();
                                        });

                                        if(typeSelection_->currentText() == "Has label"){
                                            getPipe("trigger")->flush(result != detections.end());
                                        }else if(typeSelection_->currentText() == "Has not label"){
                                            getPipe("trigger")->flush(result == detections.end());
                                        }
                                    }
                                    idle_ = true;
                                }
            );
        }


        QWidget * BlockTriggerDetection::customWidget(){
            QGroupBox *gb = new QGroupBox();
            QHBoxLayout *l = new QHBoxLayout();
            gb->setLayout(l);
            l->addWidget(typeSelection_);
            l->addWidget(labelSelection_);
            return gb;
        }
    }
}

#endif