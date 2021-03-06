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


#include <mico/core/flow/BlockVectorSplitter.h>
#include <flow/Outpipe.h>
#include <flow/Policy.h>

#include <cmath>

#include <QSpinBox>
#include <QLabel>


namespace mico{
    namespace core{
        BlockVectorSplitter::BlockVectorSplitter(){
            createPolicy({  flow::makeInput<std::vector<float>>("vector") });
            
            registerCallback({"vector"},
                                    [&](flow::DataFlow _data){
                                        auto v = _data.get<std::vector<float>>("vector");
                                        
                                        for(unsigned i = 0; i < nTrajs_; i++){
                                            getPipe("v" +std::to_string(i))->flush(v[i]);
                                        }
                                    }
            );
        }


        void BlockVectorSplitter::preparePolicy(){
            removePipes();
            for(unsigned i = 0; i < nTrajs_; i++){
                createPipe<float>("v" +std::to_string(i));
            }
        }


        QBoxLayout * BlockVectorSplitter::creationWidget(){
            QBoxLayout *layout = new QVBoxLayout();
            
            
            layout->addWidget(new QLabel("Select number of trajectories to be displayed."));

            spinBox_ = new QSpinBox();
            spinBox_->setMinimum(1);
            spinBox_->setMaximum(10);
            layout->addWidget(spinBox_);
            QWidget::connect(spinBox_, QOverload<int>::of(&QSpinBox::valueChanged), [this](int _n){ 
                    this->nTrajs_ = _n; 
                    this->preparePolicy();
                });
            return layout;
        }
    }
}