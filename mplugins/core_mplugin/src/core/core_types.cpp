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


#include <mico/core/TemplatesConversion.h>
#include <vector>
using std::vector;
//
//#ifdef _WIN32
//	// https://stackoverflow.com/a/12229732/1304903
//	// In windows, there is not proper dynamic loading of libraries so the compiler complains about the undefined definicion of the
//	// static factory registration.... As what we will do is to load the registration types and block creation, we will just define the 
//	// variable to allow the generation of the shared library to be loaded by flow_kids.
//	std::vector<std::string> flow::TypeLog::registeredTypes_ = {};
//#endif

const auto directConversionFloatInt     = directConversionType<float, int>;
const auto directConversionFloatBool    = directConversionType<float, bool>;
const auto directConversionIntFloat     = directConversionType<int, float>;
const auto directConversionIntBool      = directConversionType<int, bool>;
const auto directConversionBoolInt      = directConversionType<bool, int>;
const auto directConversionBoolFloat    = directConversionType<bool, float>;

typedef vector<int> vectorInteger;
typedef vector<float> vectorFloat;
const auto vectorIntToVectorFloat		= vectorToVector<vectorInteger, vectorFloat>;
const auto vectorFloatToVectorIntt	= vectorToVector<vectorFloat, vectorInteger>;


const auto floatToVectorFloat			= numberToVector<float, vectorFloat>;
const auto intToVectorInt				= numberToVector<int, vectorInteger>;


FLOW_CONVERSION_REGISTER(float, int, directConversionFloatInt)
FLOW_CONVERSION_REGISTER(float, bool, directConversionFloatBool)
FLOW_CONVERSION_REGISTER(int, float, directConversionIntFloat)
FLOW_CONVERSION_REGISTER(int, bool, directConversionIntBool)
FLOW_CONVERSION_REGISTER(bool, int, directConversionBoolInt)
FLOW_CONVERSION_REGISTER(bool, float, directConversionBoolFloat)
FLOW_CONVERSION_REGISTER(vectorInteger, vectorFloat, vectorIntToVectorFloat)
FLOW_CONVERSION_REGISTER(vectorFloat, vectorInteger, vectorFloatToVectorIntt)
FLOW_CONVERSION_REGISTER(float, vectorFloat, floatToVectorFloat)
FLOW_CONVERSION_REGISTER(int, vectorInteger, intToVectorInt)

