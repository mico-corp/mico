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


#include <mico/visualizers/flow/BlockHistogramVisualizer.h>

#include <flow/Policy.h>

#include <QLabel>
#include <QTimer>
#include <QPixmap>
#include <mico/visualizers/qcustomplot.h>
namespace mico{
    namespace visualizer{
        BlockHistogramVisualizer::BlockHistogramVisualizer(){
            createPolicy({  flow::makeInput<std::vector<float>>("histogram") });

            registerCallback({ "histogram" },
                &BlockHistogramVisualizer::policyCallback,
                this
            );
        }
        
        BlockHistogramVisualizer::~BlockHistogramVisualizer() {
            if (refresher_) refresher_->stop();
            if (plot_) {
                plot_->hide();
                delete plot_;
            }
        }

        bool BlockHistogramVisualizer::configure(std::vector<flow::ConfigParameterDef> _params) {
            if (!plot_) {
                plot_ = new QCustomPlot();
                plot_->setGeometry(0, 0, 400, 400);
                plot_->setWindowFlags(Qt::WindowStaysOnTopHint);
                plot_->show();
                plot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom );

                barPlot_ = new QCPBars(plot_->xAxis, plot_->yAxis);
                barPlot_->setData(xData_, yData_, true);
                refresher_ = new QTimer();
                QObject::connect(refresher_, &QTimer::timeout, [&]() { 
                    std::lock_guard<std::mutex> lock(dataLock_);
                    if (autoScale_)
                        barPlot_->rescaleAxes();
                    else
                        barPlot_->rescaleKeyAxis();

                    plot_->replot();
                    plot_->update(); 
                });
                refresher_->start(50);
            }

            if (auto param = getParamByName(_params, "AutoScale"); param) {
                autoScale_ = param.value().asBool();
            }
            return true;
        }


        std::vector<flow::ConfigParameterDef> BlockHistogramVisualizer::parameters() {
            return {
                {"AutoScale", flow::ConfigParameterDef::eParameterType::BOOLEAN, true}
            };
        }

        void BlockHistogramVisualizer::policyCallback(std::vector<float> _histogram) {
            if (yData_.size() != int(_histogram.size())) {
                std::lock_guard<std::mutex> lock(dataLock_);
                yData_.resize(int(_histogram.size()));
            }

            auto it1 = _histogram.begin();
            auto it2 = yData_.begin();
            while (it1 != _histogram.end()) {
                *it2 = *it1;
                it1++;
                it2++;
            }

            if (xData_.size() != yData_.size()) {
                std::lock_guard<std::mutex> lock(dataLock_);
                xData_.resize(yData_.size());
                for (int i = 0; i < yData_.size(); i++) xData_[i] = i;
            }

            std::lock_guard<std::mutex> lock(dataLock_);
            if (barPlot_)
                barPlot_->setData(xData_, yData_, true);
        }

    }
}
