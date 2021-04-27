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


#ifndef FLOW_TOOLS_BLOCKCREATOR_BLOCKTEMPLATE_H_
#define FLOW_TOOLS_BLOCKCREATOR_BLOCKTEMPLATE_H_

#include <string>
#include <vector>

namespace flow{
    namespace creator{
        class BlockTemplate{
        public:
            typedef std::pair<std::string, std::string> InputOutputInfo;
            
            void generate(const std::string &_path);

            void name(const std::string &_name);
            std::string name() const;

            void inputs(const std::vector<InputOutputInfo> &_inputs);
            std::vector<InputOutputInfo> inputs() const;

            void outputs(const std::vector<InputOutputInfo> &_inputs);
            std::vector<InputOutputInfo> outputs() const;

            void callbacks(const std::vector<std::string> &callbacks_);
            std::vector<std::string> callbacks() const;

        private:
            void generateHeader(const std::string &_path);
            void generateSource(const std::string &_path);

        private:
            std::string name_;
            std::vector<InputOutputInfo> inputs_;
            std::vector<InputOutputInfo> outputs_;
            std::vector<std::string> callbacks_;

        };

    }
}

#endif