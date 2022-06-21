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


#include <mico/imgproc/flow/BlockKernelConvolution.h>
#include <flow/Outpipe.h>

#include <cmath>

#include <sstream>

#include <QGroupBox>
#include <QTableWidget>
#include <QSpinBox>
#include <QDoubleValidator>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>

namespace mico{
    namespace imgproc{
        BlockKernelConvolution::BlockKernelConvolution(){
            
            createPipe<cv::Mat>("image");
            createPolicy({  flow::makeInput<cv::Mat>("input") });

            registerCallback(   {"input"}, 
                                &BlockKernelConvolution::policyCallback,
                                this        
            );
        }

        void BlockKernelConvolution::policyCallback(cv::Mat _img) {
            if (getPipe("image")->registrations()) {
                cv::Mat frame = _img.clone();
                cv::Mat result;
                if (frame.channels() == 1) {
                    std::lock_guard<std::mutex> lock(dataLock_);
                    cv::filter2D(frame, result, -1, kernel_, cv::Point(-1, -1), 0.0, cv::BORDER_REPLICATE);
                }
                else {
                    std::vector<cv::Mat> channels;
                    cv::split(frame, channels);
                    for (auto& c : channels) {
                        cv::filter2D(c, c, -1, kernel_, cv::Point(-1, -1), 0.0, cv::BORDER_REPLICATE);
                    }
                    cv::merge(channels, result);
                }

                if (getPipe("image")->registrations()) {
                    getPipe("image")->flush(result);
                }
            }
        }

        QWidget* BlockKernelConvolution::customWidget() {
            QGroupBox* gb = new QGroupBox();
            QVBoxLayout* l = new QVBoxLayout();
            gb->setLayout(l);
            
            QHBoxLayout* lbt = new QHBoxLayout();
            l->addLayout(lbt);
            
            auto lbRow = new QLabel("Rows:");
            lbt->addWidget(lbRow);
            auto sbRow = new QSpinBox();
            sbRow->setValue(3);
            lbt->addWidget(sbRow);
            QObject::connect(sbRow, QOverload<int>::of(&QSpinBox::valueChanged), [&](int _rows) {table_->setRowCount(_rows); setItemValidator(); });
            

            auto lbCols = new QLabel("Cols:");
            lbt->addWidget(lbCols);
            auto sbCol = new QSpinBox();
            sbCol->setValue(3);
            lbt->addWidget(sbCol);
            QObject::connect(sbCol, QOverload<int>::of(&QSpinBox::valueChanged), [&](int _cols) {table_->setColumnCount(_cols); setItemValidator(); });



            table_ = new QTableWidget();
            l->addWidget(table_);
            table_->setRowCount(3);
            table_->setColumnCount(3);
            setItemValidator();

            auto normalizeBt = new QPushButton("Normalize Kernel");
            l->addWidget(normalizeBt);
            QObject::connect(normalizeBt, &QPushButton::clicked, [&]() {
                float acc = 0;
                for (int trow = 0; trow < table_->rowCount(); trow++) {
                    for (int tcolumn = 0; tcolumn < table_->columnCount(); tcolumn++) {
                        auto item = (QLineEdit * ) table_->cellWidget(trow, tcolumn);
                        acc += item->text().toFloat();
                    }
                }
                for (int trow = 0; trow < table_->rowCount(); trow++) {
                    for (int tcolumn = 0; tcolumn < table_->columnCount(); tcolumn++) {
                        auto item = (QLineEdit*)table_->cellWidget(trow, tcolumn);
                        auto value = item->text().toFloat();
                        item->setText(QString::number(value / acc));
                    }
                }

            });

            auto updateBt = new QPushButton("Update Kernel");
            l->addWidget(updateBt);
            QObject::connect(updateBt, &QPushButton::clicked, [&]() {

                dataLock_.lock();
                for (int trow = 0; trow < table_->rowCount(); trow++) {
                    for (int tcolumn = 0; tcolumn < table_->columnCount(); tcolumn++) {
                        auto item = (QLineEdit*)table_->cellWidget(trow, tcolumn);
                        auto value = item->text().toFloat();
                        kernel_.at<float>(trow, tcolumn) = value;
                    }
                }
                dataLock_.unlock();
            });

            return gb;
        }

        void BlockKernelConvolution::setItemValidator() {
            for (int trow = 0; trow < table_->rowCount(); trow++) {
                for (int tcolumn = 0; tcolumn <= table_->columnCount(); tcolumn++) {
                    QLineEdit* tableline = new QLineEdit;
                    tableline->setValidator(new QDoubleValidator());
                    tableline->setText("1");
                    table_->setCellWidget(trow, tcolumn, tableline);
                }
            }
            dataLock_.lock();
            kernel_ = cv::Mat::ones(table_->rowCount(), table_->columnCount(), CV_32FC1);
            dataLock_.unlock();
        }

    }
}