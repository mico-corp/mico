//---------------------------------------------------------------------------------------------------------------------
//  MICO - core plugin
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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

#ifndef MICO_CORE_TEMPLATESCONVERSION_H_
#define MICO_CORE_TEMPLATESCONVERSION_H_

#include <vector>
#include <flow/DataFlow.h>

template<typename InT_, typename OutT_>
boost::any directConversionType(boost::any &_input){
    return (OutT_) boost::any_cast<InT_>(_input);
}

// Add concepts for compile time safety
template<typename InT_, typename OutT_>
boost::any simpleFlatVectorTakeFirst(boost::any &_input){
    return (OutT_) boost::any_cast<InT_>(_input)[0];
}


// Add concepts for compile time safety
template<typename InT_, typename OutT_>
boost::any vectorToVector(boost::any& _input) {
    auto vin = boost::any_cast<InT_>(_input);
    return OutT_(vin.begin(), vin.end());
}

#endif