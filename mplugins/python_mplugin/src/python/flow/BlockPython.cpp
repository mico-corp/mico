//---------------------------------------------------------------------------------------------------------------------
//  Python MICO plugin
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

#include <chrono>
#include <iostream>
#include <fstream>

#include <Eigen/Eigen>

#include <flow/flow.h> 

#include <mico/python/flow/BlockPython.h>
#include <mico/python/ConversionUtils.h>

#include <opencv2/opencv.hpp>

namespace py = boost::python;

namespace mico {
    namespace python{

        BlockPython::BlockPython(){
            // Interface constructor
            interfaceSelector_ = new InterfaceSelectorWidget("Python interface Selector");
            interfaceSelector_->callThisIfSmthChangeInside([this](){this->prepareInterfaces();});


            // Base layout
            blockInterpreter_ = new QGroupBox("");
            blockInterpreterLayout_ = new QVBoxLayout();
            blockInterpreter_->setLayout(blockInterpreterLayout_);
            
            pythonEditor_ = new QTextEdit;
            highlighter_ = new PythonSyntaxHighlighter(pythonEditor_->document());
            blockInterpreterLayout_->addWidget(pythonEditor_);
            runButton_ = new QPushButton("play");
            blockInterpreterLayout_->addWidget(runButton_);
            
            QWidget::connect(runButton_, &QPushButton::clicked, [this]() {
                    flow::DataFlow data({}, [](flow::DataFlow _data){});
                    this->runPythonCode(data, false);
                });

        }
        BlockPython::~BlockPython(){

        }


        QBoxLayout * BlockPython::creationWidget(){
            QBoxLayout *layout = new QVBoxLayout();
            layout->addWidget(interfaceSelector_);
            return layout;
        }

        void replaceAll(std::string& str, const std::string& from, const std::string& to) {
            if(from.empty())
                return;
            size_t start_pos = 0;
            while((start_pos = str.find(from, start_pos)) != std::string::npos) {
                str.replace(start_pos, from.length(), to);
                start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
            }
        }


        void BlockPython::prepareInterfaces(){
            // Clean previous policies and pipes if exist
            removePolicy();
            removePipes();


            outputInfo_ = interfaceSelector_->getInterfaces(InterfaceSelectorWidget::INTERFACE_TYPE::OUTPUT);
            for(auto &output: outputInfo_){
                if (output.second == typeid(int).name()) {
                    createPipe<int>(output.first);
                }else if (output.second == typeid(float).name()) {
                    createPipe<float>(output.first);
                }else if (output.second == typeid(cv::Mat).name()) {
                    createPipe<cv::Mat>(output.first);
                }
            }

            inputInfo_ = interfaceSelector_->getInterfaces(InterfaceSelectorWidget::INTERFACE_TYPE::INPUT);
            std::vector<flow::PolicyInput*> policies;
            for (auto& input: inputInfo_) {
                policies.push_back(new flow::PolicyInput{ input.first, input.second });
            }

            if(policies.size() > 0){
                createPolicy(policies);

                std::vector<std::string> inTags;
                for(auto input: inputInfo_){
                    inTags.push_back(input.first);
                }

                registerCallback(inTags, [&](flow::DataFlow _data){
                    this->runPythonCode(_data, true);
                });
            }

        }

        void BlockPython::runPythonCode(flow::DataFlow _data, bool _useData){
            if(!idle_)
                return;
            
            idle_ = false;

            std::string pythonCode = pythonEditor_->toPlainText().toStdString();
            
            try {
                // Py_Initialize();
                // py::numpy::initialize();

                py::dict locals;

                if(_useData) { // Encode inputs
                    for(auto [label, type]: inputInfo_){
                        encodeInput(locals, _data, label, type);
                    }
                }

                py::object main_module = py::import("__main__");
                py::object main_namespace = main_module.attr("__dict__");
                py::object ignored = exec(pythonCode.c_str(), main_namespace, locals);

                for(auto output:outputInfo_){
                    flushPipe(locals, output.first, output.second);
                }

            } catch (py::error_already_set const& _e) {
                // handle the exception in some way
            } catch(const std::exception& _e){
                std::cout << "Catched std exception: " << _e.what() << "\n";
            }
            
            idle_ = true;
        }

        
        void BlockPython::encodeInput(py::dict &_locals, flow::DataFlow _data, std::string _tag, std::string _typeTag){
            //std::cout << _typeTag << " --> ";
            if(_typeTag == typeid(int).name()){
                _locals[_tag.c_str()] = _data.get<int>(_tag);
            //    std::cout << _data.get<int>(_tag) << std::endl;
            }else if(_typeTag == typeid(float).name()){
                _locals[_tag.c_str()] = _data.get<float>(_tag);
            }else if(_typeTag == "vec3"){
                 auto vec = _data.get<Eigen::Vector3f>(_tag);
                 py::list pylist;
                 pylist.append(vec[0]);
                 pylist.append(vec[1]);
                 pylist.append(vec[2]);
                 _locals[_tag.c_str()] = pylist;
            }else if(_typeTag == "vec4"){
                auto vec = _data.get<Eigen::Vector4f>(_tag);
                py::list pylist;
                pylist.append(vec[0]);
                pylist.append(vec[1]);
                pylist.append(vec[2]);
                pylist.append(vec[3]);
                _locals[_tag.c_str()] = pylist;
            // }else if(_typeTag == "map44"){
            //     _locals[_tag.c_str()] = _data.get<Eigen::Matrix4f>(_tag);
            }else if(_typeTag == typeid(cv::Mat).name()){
                auto image = _data.get<cv::Mat>(_tag);
                if (image.rows != 0) {
                    _locals[_tag.c_str()] = ConvertMatToNDArray(image);
                }
            }else{
                std::cout << "Type " << _typeTag << " of label "<< _tag << " is not supported yet in python block." << ".It will be initialized as none. Please contact the administrators" << std::endl;
                return;
            }

        }

        
        void BlockPython::flushPipe(py::dict &_locals , std::string _tag, std::string _typeTag){
        
            if(_locals.contains(_tag.c_str())){
                if(_typeTag == typeid(int).name()){
                    int val = py::extract<int>(_locals[_tag]);
                    getPipe(_tag)->flush(val);
                }else if(_typeTag == typeid(float).name()){
                    float val = py::extract<float>(_locals[_tag]);
                    getPipe(_tag)->flush(val);
                // }else if(_typeTag == "vec3"){
                    // getPipe(_tag)->flush(_locals[_tag.c_str()].cast<Eigen::Vector3f>());
                // }else if(_typeTag == "vec4"){
                    // getPipe(_tag)->flush(_locals[_tag.c_str()].cast<Eigen::Vector4f>());
                // }else if(_typeTag == "mat44"){
                    // getPipe(_tag)->flush(_locals[_tag.c_str()].cast<Eigen::Matrix4f>());
                }else if(_typeTag == typeid(cv::Mat).name()){
                    py::numpy::ndarray ndarr = py::numpy::from_object(_locals[_tag]);
                    cv::Mat val = ConvertNDArrayToMat(ndarr);
                    getPipe(_tag)->flush(val);
                }else{
                    std::cout << "Type " << _typeTag << " of label "<< _tag << " is not supported yet in python block." << ".It will be initialized as none. Please contact the administrators" << std::endl;
                }
            }
        }
    }
}
