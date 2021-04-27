//---------------------------------------------------------------------------------------------------------------------
//  FLOW
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


#include <block_creator/InputOutputItemWidget.h>

#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QHBoxLayout>

namespace flow{
    namespace creator{
        InputOutputItemWidget::InputOutputItemWidget(QWidget *_parent){
            name_ = new QLineEdit();
            name_->setPlaceholderText("name");

            type_ = new QLineEdit();
            type_->setPlaceholderText("type");

            delButton_ = new QPushButton("del");

            auto l = new QHBoxLayout();
            l->addWidget(name_);
            l->addWidget(type_);
            l->addWidget(delButton_);

            this->setLayout(l);
        }
        
        void InputOutputItemWidget::name(const std::string &_name){
            name_->setText(_name.c_str());
        }
        
        std::string InputOutputItemWidget::name() const{
            return name_->text().toStdString();
        }
    
        void InputOutputItemWidget::type(const std::string &_type){
            type_->setText(_type.c_str());
        }
        
        std::string InputOutputItemWidget::type() const{
            return type_->text().toStdString();
        }       
    }
}