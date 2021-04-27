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


#include <mico/visualizers/flow/BlockQCustomPlot.h>

#include <flow/Policy.h>

#include <QLabel>
#include <QTimer>
#include <QPixmap>
#include <mico/visualizers/qcustomplot.h>
namespace mico{
    namespace visualizer{
        BlockQCustomPlot::BlockQCustomPlot(){
            plot_ = new QCustomPlot();
            t0_ = std::chrono::steady_clock::now();
            QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
            timeTicker->setTimeFormat("%h:%m:%s");
            plot_->xAxis->setTicker(timeTicker);

            plot_->xAxis->setLabel("time");
            plot_->yAxis->setLabel("value");

            plot_->setInteractions(   QCP::iRangeDrag | QCP::iRangeZoom  | QCP::iSelectPlottables );

            plot_->addGraph()->setPen(QPen(QColor(255,0,0)));;
            plot_->addGraph()->setPen(QPen(QColor(0,255,0)));;
            plot_->addGraph()->setPen(QPen(QColor(0,0,255)));;

            dataTimer_ = new QTimer();
            QObject::connect(dataTimer_, &QTimer::timeout , [this](){this->realTimePlot();});
            dataTimer_->start(30);

            createPolicy({  flow::makeInput<float>("signal1"), 
                            flow::makeInput<float>("signal2"), 
                            flow::makeInput<float>("signal3") });

            registerCallback({ "signal1" },
                [&](flow::DataFlow  _data) {
                    float data = _data.get<float>("signal1");
                    dataLock_.lock();
                    pendingData1_.push_back(data);
                    dataLock_.unlock();
                }
            );
            registerCallback({ "signal2" },
                [&](flow::DataFlow  _data) {
                    if (idle_) {
                        idle_ = false;

                        float data = _data.get<float>("signal2");
                        dataLock_.lock();
                        pendingData2_.push_back(data);
                        dataLock_.unlock();
                        idle_ = true;
                    }

                }
            );
            registerCallback({ "signal3" },
                [&](flow::DataFlow  _data) {
                    if (idle_) {
                        idle_ = false;

                        float data = _data.get<float>("signal3");
                        dataLock_.lock();
                        pendingData3_.push_back(data);
                        dataLock_.unlock();
                        idle_ = true;
                    }

                }
            );


            plot_->show();

        }
        
        BlockQCustomPlot::~BlockQCustomPlot() {
            dataTimer_->stop();
            plot_->hide();
        };

        //---------------------------------------------------------------------------------------------------------------------
        void BlockQCustomPlot::realTimePlot(){
            auto t1 = std::chrono::steady_clock::now();
            double key = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0_).count()/1000.0f;

            dataLock_.lock();
            for (auto& d : pendingData1_) {
                plot_->graph(0)->addData(key, d);
            }
            pendingData1_.clear();
            for (auto& d : pendingData2_) {
                plot_->graph(1)->addData(key, d);
            }
            pendingData2_.clear();
            for (auto& d : pendingData3_) {
                plot_->graph(2)->addData(key, d);
            }
            pendingData3_.clear();
            dataLock_.unlock();

            // ui_->w_plot->yAxis->rescale(true);
            plot_->xAxis->setRange(key, 10.0, Qt::AlignRight);

            // make key axis range scroll with the data (at a constant range size of 8):
            plot_->replot();
        }
    }
}
