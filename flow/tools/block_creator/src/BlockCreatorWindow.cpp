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


#include <block_creator/BlockCreatorWindow.h>
#include <block_creator/InputOutputItemWidget.h>
#include <block_creator/CallbackConfigWidget.h>
#include <block_creator/BlockTemplate.h>

#include "ui_BlockCreator.h"

namespace flow{
    namespace creator{
        BlockCreatorWindow::BlockCreatorWindow(){
            ui_ = new Ui::BlockCreator();
            ui_->setupUi(this);

            connect(ui_->output_add, &QPushButton::clicked, this, &BlockCreatorWindow::addOutput);
            connect(ui_->input_add, &QPushButton::clicked, this, &BlockCreatorWindow::addInput);

            ui_->inputs_list->setAlignment (Qt::AlignTop);
            ui_->outputs_list->setAlignment (Qt::AlignTop);

            ui_->callbacks_tabs->addTab(new CallbackConfigWidget(this), "Callback 1");
            ui_->callbacks_tabs->removeTab(0);

            connect(ui_->actionGenerate, &QAction::triggered, this, &BlockCreatorWindow::generateCurrent);
        }

        BlockCreatorWindow::~BlockCreatorWindow(){

        }


        void BlockCreatorWindow::addInput(){
            InputOutputItemWidget *input  = new InputOutputItemWidget(this);
            ui_->inputs_list->addWidget(input);
            inputs_.push_back(input);
            connect(input->getDelButton(), 
                    &QPushButton::clicked, 
                    std::bind([&](QWidget *_self){ 
                        inputs_.erase(std::find(inputs_.begin(), inputs_.end(),_self));
                        _self->hide();
                        ui_->inputs_list->removeWidget(_self); 
                    }, input));
        }

        void BlockCreatorWindow::addOutput(){
            InputOutputItemWidget *output  = new InputOutputItemWidget(this);
            ui_->outputs_list->addWidget(output);
            outputs_.push_back(output);
            connect(output->getDelButton(), 
                    &QPushButton::clicked, 
                    std::bind([&](QWidget *_self){ 
                        outputs_.erase(std::find(outputs_.begin(), outputs_.end(),_self));
                        _self->hide();
                        ui_->outputs_list->removeWidget(_self); 
                    }, output));
        }

        void BlockCreatorWindow::generateCurrent(){
            BlockTemplate blockTemplate;

            blockTemplate.name(ui_->block_name->text().toStdString());

            std::vector<BlockTemplate::InputOutputInfo> outputs;
            for(auto output: outputs_){
                outputs.push_back(std::make_pair(output->name(), output->type()));
            }
            blockTemplate.outputs(outputs);

            std::vector<BlockTemplate::InputOutputInfo> inputs;
            for(auto input: inputs_){
                inputs.push_back(std::make_pair(input->name(), input->type()));
            }
            blockTemplate.inputs(inputs);

            std::vector<std::string> callbacks;
            // 666 make this better
            CallbackConfigWidget* ccw = (CallbackConfigWidget*) ui_->callbacks_tabs->currentWidget();
            callbacks.push_back(ccw->callback());
            blockTemplate.callbacks(callbacks);


            blockTemplate.generate("");
        }

        void BlockCreatorWindow::loadTemplate(){
            
        }

    }
}