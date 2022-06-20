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


#include <mico/cameras/flow/BlockCalibrationMonocular.h>
#include <flow/Outpipe.h>

#include <QSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QProgressBar>
#include <QTimer>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>

namespace mico {
    namespace cameras {
        BlockCalibrationMonocular::BlockCalibrationMonocular() {
            createPolicy({ flow::makeInput<cv::Mat>("Image") });

            registerCallback({ "Image" },
                &BlockCalibrationMonocular::policyCallback,
                this
            );

        }


        BlockCalibrationMonocular::~BlockCalibrationMonocular() {
            if (imageRefresher_) {
                imageRefresher_->stop();
                delete imageRefresher_;
            }
            if (imageView_) {
                imageView_->hide();
                delete imageView_;
            }
        };



        QWidget* BlockCalibrationMonocular::customWidget() {
            auto w = new QWidget();
            auto ml = new QVBoxLayout;
            w->setLayout(ml);

            {
                auto l = new QHBoxLayout;
                ml->addLayout(l);
                l->addWidget(new QLabel("Horizontal Size: "));
                auto* whSize = new QSpinBox();
                whSize->setValue(hSize_);
                l->addWidget(whSize);
                QObject::connect(whSize, QOverload<int>::of(&QSpinBox::valueChanged), [&](int _val) { hSize_ = _val; });
                progressX_ = new QProgressBar();
                ml->addWidget(progressX_);
                progressX_->setMinimum(0);
                progressX_->setMaximum(12);
            }
            {
                auto l = new QHBoxLayout;
                ml->addLayout(l);
                l->addWidget(new QLabel("Vertical Size: "));
                auto* wvSize = new QSpinBox();
                wvSize->setValue(vSize_);
                l->addWidget(wvSize);
                QObject::connect(wvSize, QOverload<int>::of(&QSpinBox::valueChanged), [&](int _val) { vSize_ = _val; });
                progressY_ = new QProgressBar();
                ml->addWidget(progressY_);
                progressY_->setMinimum(0);
                progressY_->setMaximum(12);
            }
            {
                auto l = new QHBoxLayout;
                ml->addLayout(l);
                l->addWidget(new QLabel("Square Size (mm): "));
                auto* wSize = new QSpinBox();
                wSize->setValue(mmSize_);
                l->addWidget(wSize);
                QObject::connect(wSize, QOverload<int>::of(&QSpinBox::valueChanged), [&](int _val) { mmSize_ = _val; });
                progressSize_ = new QProgressBar();
                ml->addWidget(progressSize_);
                progressSize_->setMinimum(0);
                progressSize_->setMaximum(6);
            }

            auto* btCalibrate = new QPushButton("Calibrate");
            ml->addWidget(btCalibrate);
            QObject::connect(btCalibrate, &QPushButton::clicked, [&]() {calibrate(); });

            auto* btStartReset= new QPushButton("Start/reset");
            ml->addWidget(btStartReset);
            QObject::connect(btStartReset, &QPushButton::clicked, [&]() {startReset(); });

            
            return w;
        };


        void BlockCalibrationMonocular::startReset() {
            qualityX_ = std::vector<uint8_t>(12, 0);
            qualityY_ = std::vector<uint8_t>(12, 0);
            qualityScale_ = std::vector<uint8_t>(6, 0);

            progressX_->setValue(0);
            progressY_->setValue(0);
            progressSize_->setValue(0);

            calibPoints_.clear();

            // Think about moving lines below to another place
            if (!imageView_) {
                imageView_ = new QLabel();
                imageView_->setScaledContents(true);
                imageView_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
                imageView_->setMinimumHeight(150);
                imageView_->setMinimumWidth(150);
                imageView_->setGeometry(0, 0, 640, 480);
                imageView_->setWindowFlags(Qt::WindowStaysOnTopHint);
            }
            imageView_->show();

            if (!imageRefresher_) {
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
                        }
                        else if (image.channels() == 3) {
                            qimg = QImage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888).rgbSwapped();
                        }
                        else if (image.channels() == 4) {
                            qimg = QImage(image.data, image.cols, image.rows, image.step, QImage::Format_RGBA8888).rgbSwapped();
                        }
                        imageView_->setPixmap(QPixmap::fromImage(qimg));
                    }

                    progressX_->setValue(std::accumulate(qualityX_.begin(), qualityX_.end(), 0));
                    progressY_->setValue(std::accumulate(qualityY_.begin(), qualityY_.end(), 0));
                    progressSize_->setValue(std::accumulate(qualityScale_.begin(), qualityScale_.end(), 0));

                });
                imageRefresher_->start(30);
            }
        }

        void BlockCalibrationMonocular::computeStatistics(const std::vector<cv::Point2f>& _points, int& _size, int& _x, int& _y) {
            int minX = std::numeric_limits<int>::max();
            int maxX = 0;
            int avgX = 0;
            int avgY = 0;
            for (const auto& p : _points) {
                if (minX > p.x) minX = p.x;
                if (maxX < p.x) maxX = p.x;
                avgX += p.x;
                avgY += p.y;
            }
            _x = avgX / int(_points.size());
            _y = avgY / int(_points.size());
            _size = abs(maxX - minX);
        }

        void BlockCalibrationMonocular::checkRefineAndAdd(cv::Mat& _image, std::vector<cv::Point2f>& _points) {
            int size, x, y;
            computeStatistics(_points, size, x, y);
            int indexX = x / (_image.cols / 12);
            int indexY = y / (_image.rows / 12);
            int indexSize = size / (_image.cols / 6);

            int score = qualityX_[indexX] + qualityY_[indexY] + qualityScale_[indexSize];
            if (score != 3) {
                cv::cornerSubPix(_image, _points, cv::Size(7, 7), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
                qualityX_[indexX] = 1;
                qualityY_[indexY] = 1;
                qualityScale_[indexSize] = 1;
                calibPoints_.push_back(_points);
            }
        }

        void BlockCalibrationMonocular::calibrate() {
            if (calibPoints_.size() < 12) {
                QMessageBox::warning(nullptr, "Warning", "Cannot create a calibration with current set of imaages, try to improve the dataset");
                return;
            }
            std::cout << "Computing parameters" << std::endl;
            cv::Mat matrix = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat distorsion = cv::Mat::zeros(8, 1, CV_64F);

            std::vector<std::vector<cv::Point3f>> points3d(1);
            for (int i = 0; i < vSize_; i++) {
                for (int j = 0; j < hSize_; j++) {
                    points3d[0].push_back(cv::Point3f(float(j * mmSize_), float(i * mmSize_), 0));
                }
            }
            points3d.resize(calibPoints_.size(), points3d[0]);

            std::vector<cv::Mat> rotVectors, transVectors;

            double rmsLeft = calibrateCamera(points3d, calibPoints_, cv::Size(lastImage_.cols, lastImage_.rows), matrix, distorsion, rotVectors, transVectors, cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
            std::cout << "Rms  calibration: " << rmsLeft << std::endl;

            QString filename = QFileDialog::getSaveFileName(Q_NULLPTR, "Choose filename", "", "YAML Files (*.yml)", Q_NULLPTR, QFileDialog::DontUseNativeDialog);
            std::cout << "Stereo Camera calibrated! Saving files." << std::endl;
            cv::FileStorage fs(filename.toStdString(), cv::FileStorage::WRITE);

            fs << "Matrix" << matrix;
            fs << "DistCoeffs" << distorsion;
            fs << "rs" << rotVectors;
            fs << "ts" << transVectors;

            std::cout << "Saved files." << std::endl;
        }

	void BlockCalibrationMonocular::policyCallback(cv::Mat  _image) {

            cv::Mat image = _image;
            if (image.rows != 0) {

                std::vector<cv::Point2f> points;
                cv::Mat gray;
                cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
                bool found = cv::findChessboardCorners(gray, cv::Size(hSize_, vSize_), points, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK);

                if(found)
                    drawChessboardCorners(image, cv::Size(hSize_, vSize_), points, found);

                imgLock_.lock();
                lastImage_ = image;
                imgLock_.unlock();

                if(found)
                    checkRefineAndAdd(gray, points);
            }

        }

    }
}
