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

#if defined(INITIALIZE_PYTHON)
#   include <boost/python.hpp>
#   include <boost/python/numpy.hpp>
#endif
#include <flow/visual/FlowVisualInterface.h>
#include <flow/visual/blocks/FlowVisualBlock.h>
#include <flow/DataFlow.h>

#include <flow/flow.h>
#include <string>

#include <sstream>

using namespace flow;



int main(int _argc, char **_argv){

    #if defined(INITIALIZE_PYTHON)  // 666 Ugly workaround to load dynamically numpy libraries. For some reason, it fails finding python when loading numpy. 
                                    // The error is common, but I all that I find is for applications using python and numpy directly, not a 
                                    // library that is loaded by other app that does not have python I tried. The problem is stated here https://stackoverflow.com/a/56018943 ,
                                    // But the solution does not work in my case, probably because of the indirect usage of numpy inside of a
                                    // library, loaded dynamically by an app without python. Apart from this, the cmakelists file has been modified
        Py_Initialize();
        boost::python::numpy::initialize();
    #endif

    FlowVisualInterface interface;
    interface.init(_argc, _argv);
}