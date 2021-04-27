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


#include <mico/visualizers/flow/BlockScatterPlot.h>

#include <flow/Policy.h>

#include <QLabel>
#include <QTimer>
#include <QPixmap>
#include <mico/visualizers/qcustomplot.h>

namespace mico{
    namespace visualizer{
        BlockScatterPlot::BlockScatterPlot(){
            plot_ = new QCustomPlot();

            plot_->xAxis->setLabel("x");
            plot_->yAxis->setLabel("y");

            plot_->setInteractions(   QCP::iRangeDrag | QCP::iRangeZoom  | QCP::iSelectPlottables );

            plot_->addGraph()->setPen(QPen(QColor(255,0,0)));;
            plot_->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));
            //      QCPScatterStyle( QCPScatterStyle::ssDot, 1 ) );
            plot_->graph(0)->setLineStyle(QCPGraph::lsNone);
            // plot_->graph(0)->setAdaptiveSampling(false);

            dataTimer_ = new QTimer();
            QObject::connect(dataTimer_, &QTimer::timeout , [this](){this->realTimePlot();});
            dataTimer_->start(30);

            createPolicy({  flow::makeInput<float>("x"), 
                            flow::makeInput<float>("y")});

            registerCallback({ "x", "y" },
                [&](flow::DataFlow  _data) {
                    float x = _data.get<float>("x");
                    float y = _data.get<float>("y");
                    dataLock_.lock();
                    pendingData_.push_back(std::make_pair(x,y));
                    dataLock_.unlock();
                }
            );


            plot_->show();

        }
        
        BlockScatterPlot::~BlockScatterPlot() {
            dataTimer_->stop();
            plot_->hide();
        };

        //---------------------------------------------------------------------------------------------------------------------
        void BlockScatterPlot::realTimePlot(){

            dataLock_.lock();
            for (auto& [x,y] : pendingData_) {
                plot_->graph(0)->addData(x, y);
            }
            pendingData_.clear();
            dataLock_.unlock();

            // make key axis range scroll with the data (at a constant range size of 8):
            plot_->replot();
        }
    }
}
