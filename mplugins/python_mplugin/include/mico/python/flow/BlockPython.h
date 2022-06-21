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


#ifndef MICO_FLOW_BLOCKPYTHON_H_
#define MICO_FLOW_BLOCKPYTHON_H_

#include <flow/Block.h>

#include <QTextEdit>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QPushButton>
#include <QtWidgets>
#include <mico/python/PythonSyntaxHighlighter.h>
#include <mico/python/flow/InterfaceSelectorWidget.h>

#ifdef slots
#undef slots
#endif

#include <Python.h>
#include <boost/python.hpp>
#include <boost/python/dict.hpp>
//#include <numpy/arrayobject.h>

#include <atomic>

namespace mico{
    namespace python{ 
        /// Mico block that allows to integrate python scripts into the flow modular system..
        /// @ingroup  mico_python
        class BlockPython: public flow::Block {
        public:
            /// Get name of block
            std::string name() const override {return "Python";}
            
            /// Retreive icon of block    
            QIcon icon() const override { 
                return QIcon((flow::Persistency::resourceDir()/"python"/"Python.png").string().c_str());
            }

            /// Base constructor
            BlockPython();

            /// Base destructor
            ~BlockPython();

            /// Get custom view widget to be display in the graph
            QWidget * customWidget(){
                return blockInterpreter_;
            }

            /// Return the custom creation widget, in which the user chooses the number and type of inputs
            QBoxLayout * creationWidget() override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// This block is resizable, it returns true.
            bool resizable() const override { return true; }

        private:
            void prepareInterfaces();

            void runPythonCode(flow::DataFlow _data, bool _useData);

            void encodeInput(boost::python::dict &locals, flow::DataFlow _data, std::string _tag, std::string _typeTag);
            void flushPipe(boost::python::dict &_locals, std::string _tag, std::string _typeTag);

        private:
            inline static bool isInitialized_ = false;

            bool isReady_ = false;

            InterfaceSelectorWidget *interfaceSelector_;

            std::map<std::string, std::string>  inputInfo_, outputInfo_;
            
            QGroupBox *blockInterpreter_;
            QVBoxLayout *blockInterpreterLayout_;
            QTextEdit * pythonEditor_;
            QPushButton * runButton_;
            PythonSyntaxHighlighter *highlighter_;

        };
    }

}

#endif