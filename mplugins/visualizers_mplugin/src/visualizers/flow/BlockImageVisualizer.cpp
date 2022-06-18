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
#include <QPushButton>

namespace mico{
    namespace visualizer{
        BlockImageVisualizer::BlockImageVisualizer(){
            
            createPolicy({  flow::makeInput<cv::Mat>("Image") });

            registerCallback<cv::Mat>({"Image"}, 
                                    [&](cv::Mat _image){
                                        if(idle_){
                                            idle_ = false;  
                                            
                                            if(_image.rows != 0){
                                                imgLock_.lock();
                                                lastImage_ = _image;
                                                imgLock_.unlock();
                                            }
                                            idle_ = true;
                                        }

                                    }
                                );
        }
        
        BlockImageVisualizer::~BlockImageVisualizer() {
            if(imageRefresher_){
                imageRefresher_->stop();
                delete imageRefresher_;
            }
            if(imageView_){
                imageView_->hide();
                delete imageView_;
            } 
        };
        
        QWidget* BlockImageVisualizer::customWidget() {
            reopenButton_ = new QPushButton("Open Viewer");

            QObject::connect(reopenButton_, &QPushButton::clicked, [&]() {
                if (imageView_) imageView_->show();
            });

            return reopenButton_;
        }

        bool BlockImageVisualizer::configure(std::vector<flow::ConfigParameterDef> _params) {
            imageView_ = new QLabel();
            imageView_->setScaledContents(true);
            imageView_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
            imageView_->setMinimumHeight(150);
            imageView_->setMinimumWidth(150);
            imageView_->setGeometry(0, 0, 640, 480);
            imageView_->setWindowFlags(Qt::WindowStaysOnTopHint);
            imageView_->show(); 

            imageRefresher_ = new QTimer();

            QObject::connect(imageRefresher_, &QTimer::timeout, [&]() {
                cv::Mat image;
                imgLock_.lock();
                image = lastImage_;
                imgLock_.unlock();
                if (image.rows != 0) {
                    QImage qimg;
                    if (image.channels() == 1) {
                        qimg = QImage(image.data, image.cols, image.rows, QImage::Format_Grayscale8);
                    } else if (image.channels() == 3) {
                        qimg = QImage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888).rgbSwapped();
                    } else if (image.channels() == 4) {
                        qimg = QImage(image.data, image.cols, image.rows, image.step, QImage::Format_RGBA8888).rgbSwapped();
                    }
                    imageView_->setPixmap(QPixmap::fromImage(qimg));
                }
            });
            imageRefresher_->start(30);

            return true;

        }
    }

}   
