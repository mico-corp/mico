
//---------------------------------------------------------------------------------------------------------------------
//  flow
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

#ifndef FLOW_VISUAL_BLOCKS_FLOWBLOCK_H_
#define FLOW_VISUAL_BLOCKS_FLOWBLOCK_H_

#include <flow/flow.h>
#include <flow/Export.h>

#include <flow/visual/blocks/ParameterWidget.h>

#include <nodes/NodeDataModel>
#include <nodes/Connection>

#include <QtCore/QObject>
#include <QtWidgets/QLineEdit>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QToolButton>

#include <iostream>

using QtNodes::NodeData;
using QtNodes::NodeDataModel;
using QtNodes::NodeDataType;
using QtNodes::PortIndex;
using QtNodes::PortType;
using QtNodes::Connection;


namespace flow{
    /// Forward declaration
    class Outpipe;

    /// Interface to allow dynamic cast of MicoBlocks.
    class ConfigurableBlock{
    public:
        virtual void configure() = 0;
    };

    /// Interface to allow dynamic cast of MicoBlocks.
    class RunnableBlock{
    public:
        virtual void run() = 0;
        virtual void stop() = 0;
    private:
        bool isRunable_;
    };

    /// 
    template<typename Block_, bool HasAutoLoop_ = false>
    class FlowVisualBlock : public NodeDataModel, public ConfigurableBlock, public RunnableBlock {
    public:
        FlowVisualBlock();

        virtual ~FlowVisualBlock();
        
        QJsonObject save() const override;
        void restore(QJsonObject const &p) override;

        std::vector<flow::ConfigParameterDef> extractParamsGui();

        void configure() override;

        flow::Block * internalBlock() const; 

        bool resizable() const override { return flowBlock_->resizable(); }

        QString description() const override {return flowBlock_->description().c_str();};
    
        QIcon icon() const override { return flowBlock_->icon();};

        QBoxLayout * creationWidget() const override { return flowBlock_->creationWidget();};

        void run() override;

        void stop() override;

    public:
        QString caption() const override { return flowBlock_->name().c_str(); }

        bool captionVisible() const override { return true; }

        QString name() const override { return flowBlock_->name().c_str(); }
    
        virtual void inputConnectionDeleted(Connection const&) override;

    public:
        unsigned int nPorts(PortType portType) const override;

        NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

        std::shared_ptr<NodeData> outData(PortIndex port) override;

        void setInData(std::shared_ptr<NodeData> data, PortIndex port) override;

        QWidget * embeddedWidget() override { return configBox_; }

    private:
        std::shared_ptr<Block_> flowBlock_;
        //std::unordered_map<std::string, Outpipe*> connectedPipes_;
        std::vector<ParameterWidget*> configParams_;
        QVBoxLayout *configsLayout_  = nullptr;
        QHBoxLayout *freqsLayout_  = nullptr;
        QGroupBox *configBox_ = nullptr;
        QToolButton *configButton_  = nullptr;
        QIcon   *configStateIcon_ = nullptr;
        QCheckBox *streamActionButton_ = nullptr;
    };
}


#include <flow/visual/blocks/FlowVisualBlock.inl>

#endif