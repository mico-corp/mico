
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


#include <flow/visual/data_types/StreamerPipeInfo.h>
#include <QJsonArray>
#include <QIcon>
#include <memory>

namespace flow{

    template<typename Block_, bool HasAutoLoop_>
    inline FlowVisualBlock<Block_,HasAutoLoop_>::FlowVisualBlock() {
        flowBlock_ = std::make_shared<Block_>();

        if(flowBlock_->customWidget() == nullptr && flowBlock_->parameters().size() == 0 && !HasAutoLoop_){
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
        if(flowBlock_->parameters().size() > 0){
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
        if(HasAutoLoop_){
            streamActionButton_ = new QCheckBox("Run");
            configsLayout_->addWidget(streamActionButton_);
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

    template<typename Block_, bool HasAutoLoop_>
    inline FlowVisualBlock<Block_,HasAutoLoop_>::~FlowVisualBlock(){

    }


    template<typename Block_, bool HasAutoLoop_>
    inline QJsonObject FlowVisualBlock<Block_,HasAutoLoop_>::save() const{
        QJsonObject modelJson = NodeDataModel::save();

        unsigned counter = 0;
        QJsonObject jsonParams;
        for(const auto &param: flowBlock_->parameters()){
            QJsonObject jParam;
            jParam["type"] = configParams_[counter]->type();
            switch (configParams_[counter]->type()) {
                case flow::ConfigParameterDef::eParameterType::STRING:
                    jParam["value"] = configParams_[counter]->getParam().asString().c_str();
                    break;
                case flow::ConfigParameterDef::eParameterType::DECIMAL:
                    jParam["value"] = configParams_[counter]->getParam().asDecimal();
                    break;
                case flow::ConfigParameterDef::eParameterType::INTEGER:
                    jParam["value"] = configParams_[counter]->getParam().asInteger();
                    break;
                case flow::ConfigParameterDef::eParameterType::BOOLEAN:
                    jParam["value"] = configParams_[counter]->getParam().asBool();
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

        modelJson["autoloop"] = HasAutoLoop_;    

        return modelJson;
    }
 
    template<typename Block_, bool HasAutoLoop_>
    inline std::vector<flow::ConfigParameterDef> FlowVisualBlock<Block_,HasAutoLoop_>::extractParamsGui(){
        std::vector<flow::ConfigParameterDef> params;
        int counter = 0; 
        for(auto &param: configParams_){
            params.push_back(param->getParam());
        }
        return params;
    }

    template<typename Block_, bool HasAutoLoop_>
    inline void FlowVisualBlock<Block_,HasAutoLoop_>::restore(QJsonObject const &_json) {
        
        unsigned counter = 0;
        for(auto &param: flowBlock_->parameters()){
            QJsonValue type = _json["params"].toObject()[param.name_.c_str()].toObject()["type"];
            QJsonValue value = _json["params"].toObject()[param.name_.c_str()].toObject()["value"];
            if (!type.isUndefined() && !value.isUndefined()) {
                switch (type.toInt()) {
                case flow::ConfigParameterDef::eParameterType::STRING:
                    configParams_[counter]->setValueString(value.toString().toStdString());
                    break;
                case flow::ConfigParameterDef::eParameterType::DECIMAL:
                    configParams_[counter]->setValueDec(value.toDouble());
                    break;
                case flow::ConfigParameterDef::eParameterType::INTEGER:
                    configParams_[counter]->setValueInt(value.toInt());
                    break;
                case flow::ConfigParameterDef::eParameterType::BOOLEAN:
                    configParams_[counter]->setValueBool(value.toBool());
                    break;
            }
                
            }
            counter++;
        }
    }


    template<typename Block_, bool HasAutoLoop_>
    inline void FlowVisualBlock<Block_,HasAutoLoop_>::configure(){
        if(flowBlock_->configure(this->extractParamsGui())){
            if(configButton_) 
                configButton_->setIcon(QIcon((Persistency::resourceDir()/"check.svg").string().c_str()));
        }else{
            if(configButton_)
                configButton_->setIcon(QIcon((Persistency::resourceDir()/"cancel.svg").string().c_str()));
        }
    }

    template<typename Block_, bool HasAutoLoop_>
    inline flow::Block * FlowVisualBlock<Block_,HasAutoLoop_>::internalBlock() const{
        return flowBlock_;
    }

    
    template<typename Block_, bool HasAutoLoop_>
    inline void FlowVisualBlock<Block_,HasAutoLoop_>::run() {
        if constexpr (HasAutoLoop_) {
            flowBlock_->start();
            streamActionButton_->setChecked(true);
        }
    }

    template<typename Block_, bool HasAutoLoop_>
    inline void FlowVisualBlock<Block_,HasAutoLoop_>::stop() {
        if constexpr (HasAutoLoop_) {
            flowBlock_->stop();
            streamActionButton_->setChecked(false);
        }
    }

    template<typename Block_, bool HasAutoLoop_>
    inline void FlowVisualBlock<Block_,HasAutoLoop_>::inputConnectionDeleted(Connection const&_conn) {
        // Unregister element in policy
        auto tag = flowBlock_->getPolicy()->inputTags()[_conn.getPortIndex(PortType::In)];
        flowBlock_->disconnect(tag);
        
    }

    template<typename Block_, bool HasAutoLoop_>
    inline unsigned int FlowVisualBlock<Block_,HasAutoLoop_>::nPorts(PortType portType) const {
        unsigned int result = 0;

        switch (portType) {
        case PortType::In:
            result = flowBlock_->nInputs();
            break;
        case PortType::Out:
            result = flowBlock_->nOutputs();
        default:
            break;
        }

        return result;
    }

    template<typename Block_, bool HasAutoLoop_>
    inline NodeDataType FlowVisualBlock<Block_,HasAutoLoop_>::dataType(PortType portType, PortIndex index) const {
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

    template<typename Block_, bool HasAutoLoop_>
    inline std::shared_ptr<NodeData> FlowVisualBlock<Block_,HasAutoLoop_>::outData(PortIndex index) {
        auto tag = flowBlock_->outputTags()[index];
        std::shared_ptr<StreamerPipeInfo> ptr(new StreamerPipeInfo(flowBlock_, tag));  // 666 TODO
        return ptr;
    }


    template<typename Block_, bool HasAutoLoop_>
    inline void FlowVisualBlock<Block_,HasAutoLoop_>::setInData(std::shared_ptr<NodeData> data, PortIndex port) {
        // 666 Connections do not transfer data but streamers information to connect to internal block.
        if(data){
            auto pipeInfo = std::dynamic_pointer_cast<StreamerPipeInfo>(data)->info();
            if(pipeInfo.otherBlock_ != nullptr){
                pipeInfo.otherBlock_->connect(pipeInfo.pipeName_, flowBlock_->inputTags()[port], *flowBlock_);
            }
        }
    }

}