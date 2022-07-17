
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



#include <flow/qnodes/blocks/FlowVisualBlock.h>
#include <flow/qnodes/data_types/StreamerPipeInfo.h>
#include <QJsonArray>
#include <QIcon>
#include <memory>

namespace flow{

    FlowVisualBlock::FlowVisualBlock(std::shared_ptr<Block> _backend): flowBlock_(_backend) {

        if(flowBlock_->customWidget() == nullptr && flowBlock_->isConfigurable() == 0 && !flowBlock_->hasRunLoop()){
            configBox_ = nullptr;
            return;
        }

        // Configure group box
        configsLayout_ = new QVBoxLayout();
        configBox_ = new QGroupBox("");
        configBox_->setLayout(configsLayout_);

        // custom visualizer
        if(flowBlock_->customWidget() != nullptr){
            configsLayout_->addWidget(flowBlock_->customWidget());
        }

        // configure parameters
        if(flowBlock_->isConfigurable()){
            for(auto &param: flowBlock_->parameters()){
                configParams_.push_back(new ParameterWidget(param));
                configsLayout_->addLayout(configParams_.back());
            }
            configButton_ = new QToolButton();
            configButton_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
            configButton_->setLayoutDirection(Qt::RightToLeft);
            configButton_->setIcon(QIcon((Persistency::resourceDir()/"question.svg").string().c_str()));
            configButton_->setText("Configure");
            configsLayout_->addWidget(configButton_);
            connect(configButton_, &QPushButton::clicked, this, [this]() {
                this->configure();
            });
            //Configure configure states
            configStateIcon_ = new QIcon();
        }

        // Autoloop button
        if(flowBlock_->hasRunLoop()){
            auto layout = new QHBoxLayout();
            configsLayout_->addLayout(layout);
            layout->addWidget(new QLabel("Run"),Qt::AlignLeft);
            streamActionButton_ = new Switch();
            streamActionButton_->setFixedWidth(100);
            layout->addWidget(streamActionButton_, Qt::AlignRight);
            connect(    streamActionButton_, &QCheckBox::toggled,
                        [=](bool checked) { 
                                if(checked){
                                    flowBlock_->start();
                                }else{
                                    flowBlock_->stop();
                                }
                            });
        }


    }

    
    FlowVisualBlock::~FlowVisualBlock(){

    }


    
    QJsonObject FlowVisualBlock::save() const{
        QJsonObject modelJson = NodeDataModel::save();

        unsigned counter = 0;
        QJsonObject jsonParams;
        for(const auto &param: flowBlock_->parameters()){
            QJsonObject jParam;
            jParam["type"] = int(configParams_[counter]->type());
            switch (configParams_[counter]->type()) {
                case flow::ConfigParameterDef::eParameterType::BOOLEAN:
                    jParam["value"] = configParams_[counter]->getParam().asBool();
                    break;
                case flow::ConfigParameterDef::eParameterType::INTEGER:
                    jParam["value"] = configParams_[counter]->getParam().asInteger();
                    break;
                case flow::ConfigParameterDef::eParameterType::DECIMAL:
                    jParam["value"] = configParams_[counter]->getParam().asDecimal();
                    break;
                case flow::ConfigParameterDef::eParameterType::STRING:
                    jParam["value"] = configParams_[counter]->getParam().asString().c_str();
                    break;
                case flow::ConfigParameterDef::eParameterType::PATH:
                    jParam["value"] = configParams_[counter]->getParam().asPath().string().c_str();
                    break;
                case flow::ConfigParameterDef::eParameterType::OPTIONS:
                    jParam["value"] = configParams_[counter]->getParam().selectedOption().c_str();
                    break;
            }

            jsonParams[param.name_.c_str()] = jParam;
            counter++;
        }
        modelJson["params"] = jsonParams;

        modelJson["class"] = typeid(flowBlock_).name();

        // Save input tags
        std::vector<std::string> inTags = flowBlock_->inputTags();
        QJsonArray jsonInTags;
        for(auto &tag:inTags){
            jsonInTags.append(tag.c_str());
        }
        modelJson["in_tags"] = jsonInTags;

        // Save output tags
        std::vector<std::string> outTags = flowBlock_->outputTags();
        QJsonArray jsonOutTags;
        for(auto &tag:outTags){
            jsonOutTags.append(tag.c_str());
        }
        modelJson["out_tags"] = jsonOutTags;

        modelJson["autoloop"] = flowBlock_->hasRunLoop();

        return modelJson;
    }
 
    
    std::vector<flow::ConfigParameterDef> FlowVisualBlock::extractParamsGui(){
        std::vector<flow::ConfigParameterDef> params;
        for(auto &param: configParams_){
            params.push_back(param->getParam());
        }
        return params;
    }

    
    void FlowVisualBlock::restore(QJsonObject const &_json) {
        
        unsigned counter = 0;
        for(auto &param: flowBlock_->parameters()){
            QJsonValue type = _json["params"].toObject()[param.name_.c_str()].toObject()["type"];
            QJsonValue value = _json["params"].toObject()[param.name_.c_str()].toObject()["value"];
            if (!type.isNull() && !value.isNull()) {
                auto eType = static_cast<flow::ConfigParameterDef::eParameterType>(type.toInt());
                switch (eType) {
                case flow::ConfigParameterDef::eParameterType::BOOLEAN:
                    configParams_[counter]->setValueBool(value.toBool());
                    break;
                case flow::ConfigParameterDef::eParameterType::INTEGER:
                    configParams_[counter]->setValueInt(value.toInt());
                    break;
                case flow::ConfigParameterDef::eParameterType::DECIMAL:
                    configParams_[counter]->setValueDec(value.toDouble());
                    break;
                case flow::ConfigParameterDef::eParameterType::STRING:
                    configParams_[counter]->setValueString(value.toString().toStdString());
                    break;
                case flow::ConfigParameterDef::eParameterType::PATH:
                    configParams_[counter]->setValuePath(value.toString().toStdString());
                    break;
                case flow::ConfigParameterDef::eParameterType::OPTIONS:
                    configParams_[counter]->setValueOption(value.toString().toStdString());
                    break;
            }
                
            }
            counter++;
        }
    }


    
    void FlowVisualBlock::configure(){
        if(flowBlock_->configure(this->extractParamsGui())){
            if(configButton_) 
                configButton_->setIcon(QIcon((Persistency::resourceDir()/"check.svg").string().c_str()));
        }else{
            if(configButton_)
                configButton_->setIcon(QIcon((Persistency::resourceDir()/"cancel.svg").string().c_str()));
        }
    }

    /*
    flow::Block * FlowVisualBlock::internalBlock() const{
        return flowBlock_;
    }*/

    
    
    void FlowVisualBlock::run() {
        if (flowBlock_->hasRunLoop()) {
            flowBlock_->start();
            streamActionButton_->setChecked(true);
        }
    }

    
    void FlowVisualBlock::stop() {
        if (flowBlock_->hasRunLoop()) {
            flowBlock_->stop();
            streamActionButton_->setChecked(false);
        }
    }

    
    void FlowVisualBlock::inputConnectionDeleted(Connection const&_conn) {
        // Unregister element in policy
        auto tag = flowBlock_->getPolicy()->inputTags()[_conn.getPortIndex(PortType::In)];
        flowBlock_->disconnect(tag);
        
    }

    
    unsigned int FlowVisualBlock::nPorts(PortType portType) const {
        size_t result = 0;

        switch (portType) {
        case PortType::In:
            result = flowBlock_->nInputs();
            break;
        case PortType::Out:
            result = flowBlock_->nOutputs();
        default:
            break;
        }

        return static_cast<unsigned int>(result);
    }

    
    NodeDataType FlowVisualBlock::dataType(PortType portType, PortIndex index) const {
        std::vector<std::string> tags;
        if(portType == PortType::In){
            tags = flowBlock_->inputTags();
        }else{
            tags = flowBlock_->outputTags();
        }

        assert(unsigned(index) < tags.size());
        
        auto iter = tags.begin() + index;
        std::string tag = *iter;

        // In NodeEditor, NodeDataType is just a pair of string indicating type and name of type. So faking it.
        //return StreamerPipeInfo(nullptr, tag).type();
        
        std::string portTypeTag;
        if(portType == PortType::In){
            portTypeTag = flowBlock_->getPolicy()->type(tag);
            // std::cout << Block_::name() << ". " << index << ": " << type << std::endl;
        }else{
            portTypeTag = flowBlock_->getPipe(tag)->type();
            // std::cout << Block_::name() << ". " << index << ": " << type << std::endl;
        }

        //return NodeDataType{portTypeTag.c_str(),tag.c_str()};
        return NodeDataType{nullptr,tag.c_str()}; // Allowing connecting any with any.
    }

    
    std::shared_ptr<NodeData> FlowVisualBlock::outData(PortIndex index) {
        auto tag = flowBlock_->outputTags()[index];
        std::shared_ptr<StreamerPipeInfo> ptr(new StreamerPipeInfo(flowBlock_, tag));  // 666 TODO
        return ptr;
    }


    
    void FlowVisualBlock::setInData(std::shared_ptr<NodeData> data, PortIndex port) {
        // 666 Connections do not transfer data but streamers information to connect to internal block.
        if(data){
            auto pipeInfo = std::dynamic_pointer_cast<StreamerPipeInfo>(data)->info();
            if(pipeInfo.otherBlock_ != nullptr){
                pipeInfo.otherBlock_->connect(pipeInfo.pipeName_, flowBlock_->inputTags()[port], *flowBlock_);
            }
        }
    }

}