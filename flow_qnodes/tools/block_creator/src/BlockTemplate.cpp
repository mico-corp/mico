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

#include <block_creator/BlockTemplate.h>
#include <block_creator/TemplateHeader.h>
#include <block_creator/TemplateSource.h>

#include <QString>

#include <fstream>
#include <sstream>

namespace flow{
    namespace creator{
        void BlockTemplate::generate(const std::string &_path){
            generateHeader(_path);
            generateSource(_path);
        }

        void BlockTemplate::name(const std::string &_name){
            name_ = _name;
        }


        std::string BlockTemplate::name() const{
            return name_;
        }


        void BlockTemplate::inputs(const std::vector<BlockTemplate::InputOutputInfo> &_inputs){
            inputs_ = _inputs;
        }


        std::vector<BlockTemplate::InputOutputInfo> BlockTemplate::inputs() const{
            return inputs_;
        }


        void BlockTemplate::outputs(const std::vector<BlockTemplate::InputOutputInfo> &_outputs){
            outputs_ = _outputs;
        }


        std::vector<BlockTemplate::InputOutputInfo> BlockTemplate::outputs() const{
            return outputs_;
        }


        void BlockTemplate::callbacks(const std::vector<std::string> &_callbacks){
            callbacks_ = _callbacks;
        }


        std::vector<std::string> BlockTemplate::callbacks() const{
            return callbacks_;
        }


        void BlockTemplate::generateHeader(const std::string &_path){
            QString formattedHeader(templateHeader.c_str());

            formattedHeader= formattedHeader    .arg(name_.c_str())
                                                .arg("Empty description");

            std::ofstream file(name_+".h");
            file << formattedHeader.toStdString();
        }

        void BlockTemplate::generateSource(const std::string &_path){
            // Init file
            std::ofstream file(name_+".cpp");
            file << templatePolicy;
            {
                QString formatStr(templateBasicIncludesSources.c_str());
                formatStr =  formatStr.arg(name_.c_str());
                file << formatStr.toStdString();
            }
            
            // Init constructor
            {
                QString formatStr(templateInitConstructor.c_str());
                formatStr =  formatStr.arg(name_.c_str());
                file << formatStr.toStdString();
            }

            // Create pipes
            for(auto output:outputs_){
                QString templatePipe(templatePipeCreation.c_str());
                templatePipe = templatePipe .arg(output.first.c_str())
                                            .arg(output.second.c_str());
                file << templatePipe.toStdString();
            }


            // Create policy
            std::stringstream policy;
            for(unsigned i = 0; i < inputs_.size(); i++){
                QString temInput(templateInput.c_str());
                temInput = temInput .arg(inputs_[i].first.c_str())
                                    .arg(inputs_[i].second.c_str());
                policy << temInput.toStdString();
                if(i < inputs().size()-1) policy << ",";
            }
            if(inputs_.size()!= 0){
                QString formatStr(templatePolicyCreation.c_str());
                formatStr =  formatStr.arg(policy.str().c_str());
                file << formatStr.toStdString();
            }

            // Create callbacks
            for(auto cb:callbacks_){
                QString templatePipe(cb.c_str());

                /// Capture inputs
                //QRegExp inputRx("FLOW_INPUT(\\a+, \\a+)");

                /// Capture outputs
                //QRegExp outputRx("FLOW_OUTPUT(\\a+, \\a+)");
                

                file << templatePipe.toStdString();
            }

            // End constructor
            file << templateEndConstructor;


        }



    }
}