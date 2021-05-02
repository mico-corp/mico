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


#include <mico/cameras/flow/SingleImageFlusher.h>
#include <flow/Outpipe.h>


#include <QGroupBox>
#include <QLineEdit>
#include <QPushButton>
#include <QFileDialog>


namespace mico{
    namespace cameras {
        SingleImageFlusher::SingleImageFlusher() {
            createPipe<cv::Mat>("Image");
            
            customWidget_ = new QGroupBox();

            QVBoxLayout* l = new QVBoxLayout();
            customWidget_->setLayout(l);

            auto entryLay = new QHBoxLayout();
            filePathLe_ = new QLineEdit("Write full image path");
            entryLay->addWidget(filePathLe_);

            auto browseBt = new QPushButton("Browse");
            entryLay->addWidget(browseBt);
            QObject::connect(browseBt, &QPushButton::clicked, [&]() {
                QString directory = QFileDialog::getOpenFileName(Q_NULLPTR, "Choose file", "", "All Files (*)", Q_NULLPTR, QFileDialog::DontUseNativeDialog);

                if (!directory.isEmpty()) {
                    filePathLe_->setText(directory);
                }
            });

            l->addLayout(entryLay);

            auto qPushBt = new QPushButton("Flush image");
            l->addWidget(qPushBt);

            QObject::connect(qPushBt, &QPushButton::clicked, [&](){
                auto filePath = filePathLe_->text().toStdString();
                auto image = cv::imread(filePath);
                if(image.rows != 0 && getPipe("Image")->registrations() != 0){
                    getPipe("Image")->flush(image);
                }else{
                    filePathLe_->setText("Bad Image Path");
                }
            });

        }


        SingleImageFlusher::~SingleImageFlusher() {
            if (camera_) {
                camera_->release();
                delete camera_;
            }
        };


        QWidget* SingleImageFlusher::customWidget() { 
            return customWidget_; 
        };
    }
}
