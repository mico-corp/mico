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


#include <mico/visualizers/flow/BlockImageVisualizer.h>

#include <flow/Policy.h>

#include <QLabel>
#include <QTimer>
#include <QPixmap>

namespace mico{
    namespace visualizer{
        BlockImageVisualizer::BlockImageVisualizer(){
            imageView_ = new QLabel();
            imageView_->setScaledContents(true);
            imageView_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
            imageView_->setMinimumHeight(150);
            imageView_->setMinimumWidth(150);
            imageView_->show();

            imageRefresher_ = new QTimer();

            QObject::connect(imageRefresher_, &QTimer::timeout, [&]() {
                cv::Mat image;
                imgLock_.lock();
                image = lastImage_;
                imgLock_.unlock();
                if (image.rows != 0) {
                    QImage qimg = QImage(image.data, image.cols, image.rows, QImage::Format_RGB888).rgbSwapped();
                    imageView_->setPixmap(QPixmap::fromImage(qimg));
                }
            });
            imageRefresher_->start(30);

            createPolicy({  flow::makeInput<cv::Mat>("Image") });

            registerCallback({"Image"}, 
                                    [&](flow::DataFlow  _data){
                                        if(idle_){
                                            idle_ = false;  
                                            
                                            cv::Mat image = _data.get<cv::Mat>("Image");
                                            if(image.rows != 0){
                                                imgLock_.lock();
                                                lastImage_ = image;
                                                imgLock_.unlock();
                                            }
                                            idle_ = true;
                                        }

                                    }
                                );
        }
        
        BlockImageVisualizer::~BlockImageVisualizer() {
            imageRefresher_->stop();
            imageView_->hide();
        };
    }
}   
