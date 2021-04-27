
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


#ifndef FLOW_VISUAL_FLOWVISUALINTERFACE_H_
#define FLOW_VISUAL_FLOWVISUALINTERFACE_H_

#ifdef HAS_QTNODEEDITOR

#include <flow/Export.h>

#include <nodes/DataModelRegistry>
#include <QtWidgets/QApplication>

#include <functional>

namespace QtNodes{
    class FlowScene;
}

namespace flow{

    class FlowVisualInterface{
        public:
            typedef std::function<void(std::shared_ptr<QtNodes::DataModelRegistry> &_registry)> RegistryFnType_;
            typedef std::shared_ptr<QtNodes::DataModelRegistry> RegistryType_;

            int init(int _argc, char** _argv);
            void quit();

            void setNodeRegisterFn(std::function<void(std::shared_ptr<QtNodes::DataModelRegistry> &_registry)> _fn);
            void setCodeGeneratorCustoms(   const std::vector<std::string> &_customIncludes = {},
                                            const std::vector<std::string> &_customFinds = {}, 
                                            const std::vector<std::string> &_customLinks = {} );
        private:
            std::shared_ptr<QtNodes::DataModelRegistry> registerDataModels();

        private:
            void loadCustomPlugins(std::shared_ptr<QtNodes::DataModelRegistry> &_registry);
            void configureAll();
            void runAll();
            void setStyle();

        private:
            QApplication *kids_app;
            QtNodes::FlowScene* scene;

            std::shared_ptr<QtNodes::DataModelRegistry> nodeRegistry_;
            std::function<void(std::shared_ptr<QtNodes::DataModelRegistry> &_registry)> registerFn_ = nullptr;

            // Code generator extra information
            std::vector<std::string> customIncludes_ = {};
            std::vector<std::string> customFinds_ = {};
            std::vector<std::string> customLinks_ = {};
    };
}





#endif

#endif