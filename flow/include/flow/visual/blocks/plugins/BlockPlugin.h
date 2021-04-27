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


#ifndef FLOW_PLUGINS_BLOCKPLUGIN_H_
#define FLOW_PLUGINS_BLOCKPLUGIN_H_

#include <flow/Block.h>

namespace flow{

    class PluginNodeCreator{
    public:
        using RegistryItemPtr     = std::unique_ptr<QtNodes::NodeDataModel>;
        using RegistryItemCreator = std::function<RegistryItemPtr()>;

        void registerNodeCreator(RegistryItemCreator _fn, std::string _category = "Plugin"){
            creatorFun_.push_back({_category, _fn});
        }

        std::vector<std::pair<std::string, RegistryItemCreator>> get() { return creatorFun_; };

    private:
        std::vector<std::pair<std::string, RegistryItemCreator>> creatorFun_;
    };
}

#endif