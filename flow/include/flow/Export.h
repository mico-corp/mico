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

#ifndef FLOW_EXPORT_H_
#define FLOW_EXPORT_H_

#if defined(_WIN32)
#  define FLOW_DECL_EXPORT __declspec(dllexport)
#  define FLOW_FACTORY_EXPORT __declspec(dllexport)
#  define FLOW_DECL_IMPORT __declspec(dllimport)
#elif defined (__linux__)
#  define FLOW_DECL_EXPORT
#  define FLOW_FACTORY_EXPORT
#  define FLOW_DECL_IMPORT
#endif

#if defined (FLOW_LIB_SHARED)
#  ifdef FLOW_EXPORTS
#    define FLOW_DECL FLOW_DECL_EXPORT
#  else
#    define FLOW_DECL FLOW_DECL_IMPORT
#  endif
#elif !defined (FLOW_LIB_SHARED)
#  define FLOW_DECL
#endif

#endif