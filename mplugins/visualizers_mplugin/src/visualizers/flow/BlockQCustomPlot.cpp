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
            createPolicy({  flow::makeInput<float>("signal1"), 
                            flow::makeInput<float>("signal2"), 
                            flow::makeInput<float>("signal3") });


            std::function<void(float, std::vector<std::pair<float, float>>&)> cbProto = [&](float _signal, std::vector<std::pair<float, float>>& _dataBuffer) {
                auto t1 = std::chrono::steady_clock::now();
                double key = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0_).count() / 1000.0f;
                dataLock_.lock();
                _dataBuffer.push_back({ key, _signal });
                dataLock_.unlock();
            };

            std::function<void(float)> cb1 = std::bind(cbProto, std::placeholders::_1, pendingData1_);
            registerCallback({ "signal1" }, cb1 );

            std::function<void(float)> cb2 = std::bind(cbProto, std::placeholders::_1, pendingData2_);
            registerCallback({ "signal2" }, cb1);

            std::function<void(float)> cb3 = std::bind(cbProto, std::placeholders::_1, pendingData3_);
            registerCallback({ "signal3" }, cb1);
        }
        
        BlockQCustomPlot::~BlockQCustomPlot() {
            if(dataTimer_) dataTimer_->stop();
            if(plot_) plot_->hide();
        };

        std::vector<flow::ConfigParameterDef> BlockQCustomPlot::parameters() {
            return {
                {"TrackPlot", flow::ConfigParameterDef::eParameterType::BOOLEAN, true},
                {"RangePlot", flow::ConfigParameterDef::eParameterType::DECIMAL, 10.0f}
            };
        }


        bool BlockQCustomPlot::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (!plot_) {
                plot_ = new QCustomPlot();
                t0_ = std::chrono::steady_clock::now();
                QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
                timeTicker->setTimeFormat("%h:%m:%s");
                plot_->xAxis->setTicker(timeTicker);

                plot_->xAxis->setLabel("time");
                plot_->yAxis->setLabel("value");

                plot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

                plot_->addGraph()->setPen(QPen(QColor(255, 0, 0)));;
                plot_->addGraph()->setPen(QPen(QColor(0, 255, 0)));;
                plot_->addGraph()->setPen(QPen(QColor(0, 0, 255)));;

                dataTimer_ = new QTimer();
                QObject::connect(dataTimer_, &QTimer::timeout, [this]() {this->realTimePlot(); });
                dataTimer_->start(30);

                plot_->setGeometry(0, 0, 400, 400);
                plot_->setWindowFlags(Qt::WindowStaysOnTopHint);
                plot_->show();
            }

            if (auto param = getParamByName(_params, "TrackPlot"); param) {
                trackPlot_ = param.value().asBool();
            }
            if (auto param = getParamByName(_params, "RangePlot"); param) {
                rangePlot_= param.value().asDecimal();
                plot_->xAxis->setRange(lastKey_, rangePlot_, Qt::AlignRight);
            }


            
            return true;
        }


        //---------------------------------------------------------------------------------------------------------------------
        void BlockQCustomPlot::realTimePlot(){
            dataLock_.lock();
            auto data1 = pendingData1_;
            pendingData1_.clear();
            auto data2 = pendingData2_;
            pendingData2_.clear();
            auto data3 = pendingData3_;
            pendingData3_.clear();
            dataLock_.unlock();

            for (auto& [key, data] : data1) {
                plot_->graph(0)->addData(key, data);
            }
            for (auto& [key, data] : data2) {
                plot_->graph(1)->addData(key, data);
            }
            for (auto& [key, data] : data3) {
                plot_->graph(2)->addData(key, data);
            }

            // ui_->w_plot->yAxis->rescale(true);
            if (trackPlot_) {
                if (data1.size() && data1.back().first > lastKey_ ) lastKey_  = data1.back().first;
                if (data2.size() && data2.back().first > lastKey_ ) lastKey_  = data2.back().first;
                if (data3.size() && data3.back().first > lastKey_ ) lastKey_  = data3.back().first;
                plot_->xAxis->setRange(lastKey_, rangePlot_, Qt::AlignRight);
            }

            // make key axis range scroll with the data (at a constant range size of 8):
            plot_->replot();
        }
    }
}
