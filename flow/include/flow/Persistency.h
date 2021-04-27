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


#if defined(__linux__)
    #include <experimental/filesystem>  // Not implemented until g++8
    namespace fs = std::experimental::filesystem;
#elif defined(_WIN32)
    #include <filesystem>
    namespace fs = std::filesystem;
#endif


namespace flow{

    
    /// Base class of flow that is used to get the root folders of flow
    /// @ingroup  flow
    class Persistency{
    public:
        static fs::path resourceDir () {
            #if defined(_WIN32)
                return "c:\\.flow\\plugins\\resources";
            #elif defined(__linux__)
                std::string userDir(getenv("USER"));
                std::string resourcesDir = "/home/"+userDir+"/.flow/plugins/resources";
                return resourcesDir;
            #endif
        }
    };
}

#endif