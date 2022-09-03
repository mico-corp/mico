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

#ifndef FLOW_PERSISTENCY_H_
#define FLOW_PERSISTENCY_H_

#include <string>
#include <filesystem>

namespace fs = std::filesystem;

namespace flow{

    
    /// Base class of flow that is used to get the root folders of flow
    /// @ingroup  flow
    class Persistency{
    public:
        static void setResourceDir(fs::path _path) {
            resourceDir_ = _path;
            isInit_ = true;
        }

        static fs::path resourceDir () {
            init();
            return resourceDir_;
        }

    private:
        static void init() {
            if (isInit_) return;

            isInit_ = true;
            #if defined(_WIN32)
                // 666 uf............. depend on installation? fix it? will we support x86? make it configurable
                resourceDir_ = "C:\\Program Files\\mico-corp\\mico\\bin\\mplugins\\resources";
            #elif defined(__linux__)
                std::string userDir(getenv("USER"));
                std::string resourceDir_ = "/usr/bin/mplugins/resources";
            #endif
        }

        inline static fs::path resourceDir_ = "";
        inline static bool isInit_ = false;
    };
}

#endif