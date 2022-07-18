
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

#include <flow/qnodes/blocks/ParameterWidget.h>

#include <QIntValidator>
#include <QDoubleValidator>
#include <QSpinBox>
#include <QComboBox>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QFileDialog>
#include <QGroupBox>

namespace flow{

    std::vector<std::string> split(const std::string& s, char delimiter){
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }

    //---------------------------------------------------------------------------------------------------------------------
    ParameterWidget::ParameterWidget(   const ConfigParameterDef &_param, 
                                        QWidget *_parent):  QHBoxLayout(_parent), 
                                                            param_(_param) {
        lName_ = _param.name_;
        type_ = _param.type_;
        label_ = new QLabel(_param.name_.c_str());
        this->addWidget(label_);


        switch (type_) {
        case flow::ConfigParameterDef::eParameterType::STRING:
            value_ = new QLineEdit();
            this->addWidget(value_);
            
            static_cast<QLineEdit*>(value_)->setText(_param.asString().c_str());
            break;
        case flow::ConfigParameterDef::eParameterType::DECIMAL:
            value_ = new QLineEdit();
            this->addWidget(value_);
    
            static_cast<QLineEdit*>(value_)->setText(QString::number(_param.asDecimal(), 'f', 5));
            static_cast<QLineEdit*>(value_)->setValidator( new QDoubleValidator() );
            break;
        case flow::ConfigParameterDef::eParameterType::INTEGER:
        {
            value_ = new QSpinBox();
            this->addWidget(value_);

            static_cast<QSpinBox*>(value_)->setValue(_param.asInteger());
            static_cast<QSpinBox*>(value_)->setMaximum(std::numeric_limits<int>::max());
            break;
        }
        case flow::ConfigParameterDef::eParameterType::BOOLEAN:
            value_ = new QCheckBox();
            this->addWidget(value_);

            break;
        case flow::ConfigParameterDef::eParameterType::PATH:
        {
            filePath_ = new QLineEdit("File Path");
            value_ = filePath_;
            this->addWidget(value_);

            browseButton_ = new QPushButton("Browse");
            connect(browseButton_, &QPushButton::clicked, this, &ParameterWidget::browseCallback);
            this->addWidget(browseButton_);
            
            break;
        }
        case flow::ConfigParameterDef::eParameterType::OPTIONS:
            value_ = new QComboBox();
            for(const auto &opt: _param.asOptions()){
                static_cast<QComboBox*>(value_)->addItem(opt.c_str());
            }
            this->addWidget(value_);
            break;
        }

    }

    //---------------------------------------------------------------------------------------------------------------------
    ParameterWidget::~ParameterWidget(){
        delete label_;
        delete value_;
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::string ParameterWidget::label() const{
        return label_->text().toStdString();
    }


    ConfigParameterDef ParameterWidget::getParam(){
        switch (type_){
            case ConfigParameterDef::eParameterType::BOOLEAN:
                return { lName_, type_, static_cast<QCheckBox*>(value_)->isChecked()};
                break;
            case ConfigParameterDef::eParameterType::INTEGER:
                return { lName_, type_, static_cast<QSpinBox*>(value_)->value()};
                break;
            case ConfigParameterDef::eParameterType::DECIMAL:
            {
                std::stringstream ss;
                ss << static_cast<QLineEdit*>(value_)->text().toStdString();
                float val;
                ss >> val;
                return { lName_, type_, val};
                break;
            }
            case ConfigParameterDef::eParameterType::STRING:
                return { lName_, type_, static_cast<QLineEdit*>(value_)->text().toStdString()};
                break;
            case ConfigParameterDef::eParameterType::PATH:
                return { lName_, type_,  fs::path{filePath_->text().toStdString()} };
                break;
            case ConfigParameterDef::eParameterType::OPTIONS:
            {
                std::string selected = static_cast<QComboBox*>(value_)->currentText().toStdString() ;
                return { lName_, ConfigParameterDef::eParameterType::OPTIONS, param_.asOptions(), selected };
                break;
            }
        }

        assert(false);
        return { "fail", ConfigParameterDef::eParameterType::BOOLEAN, false }; //Supposed to not to be reached
    }

    void ParameterWidget::setValueString(std::string _val){
        static_cast<QLineEdit*>(value_)->setText(_val.c_str());
    }

    void ParameterWidget::setValueInt(int _val){
        static_cast<QSpinBox*>(value_)->setValue(_val);
    }

    void ParameterWidget::setValueDec(float _val){
        static_cast<QLineEdit*>(value_)->setText(std::to_string(_val).c_str());
    }

    void ParameterWidget::setValueBool(bool _val){
        static_cast<QCheckBox*>(value_)->setChecked(_val);
    }

    void ParameterWidget::setValuePath(fs::path _val) {
        filePath_->setText(_val.string().c_str());
    }

    void ParameterWidget::setValueOption(std::string _val) {
        if (int idx = static_cast<QComboBox*>(value_)->findText(_val.c_str()); idx != -1) {
            static_cast<QComboBox*>(value_)->setCurrentIndex(idx);
        }
    }

    void ParameterWidget::browseCallback() {
        QString directory = QFileDialog::getOpenFileName(Q_NULLPTR, "Choose file", "", "All Files (*)", Q_NULLPTR, QFileDialog::DontUseNativeDialog);

        if (!directory.isEmpty()) {
            static_cast<QLineEdit*>(value_)->setText(directory);
        }
    }
}