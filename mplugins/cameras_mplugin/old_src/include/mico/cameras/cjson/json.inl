//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2015 Carmelo J. Fern�ndez-Ag�era Tortosa
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
// Simple Json C++ library
//----------------------------------------------------------------------------------------------------------------------
#ifndef _CJSON_JSON_INL_
#define _CJSON_JSON_INL_

#include "json.h" // This will actually be ignored due to guards, but works for intellisense.
#include <cassert>
#include <utility> // std::move

#include <iostream>

namespace cjson {
	//------------------------------------------------------------------------------------------------------------------
	inline Json::Json() 
		: mType (DataType::null)
	{
		//std::cout << "Json::Json " << this << "\n";
	}

	//------------------------------------------------------------------------------------------------------------------
	inline Json::Json(const Json& _x)
		: mType (_x.mType)
	{
		//std::cout << "Json::Json(const &) " << this << "\n";
		switch(mType) {
		case DataType::boolean:
		case DataType::integer:
		case DataType::real:
			mNumber = _x.mNumber;
			break;
		case DataType::text:
			mText = _x.mText;
			break;
		case DataType::array:
			mArray.reserve(_x.mArray.size());
			for(auto element : _x.mArray)
				mArray.push_back(new Json(*element));
			break;
		case DataType::object:
			for(auto element : _x.mObject)
				mObject.insert(std::make_pair(element.first, new Json(*element.second)));
			break;
		default: // Do nothing for null
			break;
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	inline Json::Json(Json&& _x)
		: mType(_x.mType)
	{
		//std::cout << "Json::Json(&&) " << this << "\n";
		switch(mType) {
		case DataType::boolean:
		case DataType::integer:
		case DataType::real:
			mNumber = _x.mNumber;
			break;
		case DataType::text:
			mText = std::move(_x.mText);
			break;
		case DataType::array:
			mArray = std::move(_x.mArray);
			_x.mType = DataType::null;
			break;
		case DataType::object:
			mObject = std::move(_x.mObject);
			_x.mType = DataType::null;
			break;
		default: // Do nothing for null
			break;
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	template<typename T_>
	Json::Json(std::initializer_list<T_> _list)
		:mType(DataType::array)
	{
		//std::cout << "Json::Json(list) " << this << "\n";
		mArray.reserve(_list.size());
		for(const auto& element : _list)
			mArray.push_back(new Json(element));
	}

	//------------------------------------------------------------------------------------------------------------------
	template<class T_>
	Json::Json(const std::vector<T_>& _list)
		:mType(DataType::array)
	{
		//std::cout << "Json::Json (vector)" << this << "\n";
		mArray.reserve(_list.size());
		for(const auto& element : _list)
			mArray.push_back(new Json(element));
	}

	//------------------------------------------------------------------------------------------------------------------
	inline Json& Json::operator=(const Json& _x)
	{
		clear();
		mType = _x.mType;
		switch(mType) {
		case DataType::boolean:
		case DataType::integer:
		case DataType::real:
			mNumber = _x.mNumber;
			break;
		case DataType::array:
			mArray = _x.mArray;
			// Deep copy
			for (size_t i = 0; i < mArray.size(); ++i) {
				mArray[i] = new Json(*mArray[i]);
			}
			break;
		case DataType::object:
			mObject = mapDeepCopy(_x.mObject);
			break;
		case DataType::text:
			mText = _x.mText;
			break;
		default: // Do nothing for null
			break;
		}
		return *this;
	}

	//------------------------------------------------------------------------------------------------------------------
	inline Json& Json::operator=(Json&& _x)
	{
		clear();
		mType = _x.mType;
		switch(mType) {
		case DataType::boolean:
		case DataType::integer:
		case DataType::real:
			mNumber = _x.mNumber;
			break;
		case DataType::array:
			mArray = std::move(_x.mArray);
			_x.mType = DataType::null;
			break;
		case DataType::object:
			mObject = std::move(_x.mObject);
			_x.mType = DataType::null;
			break;
		case DataType::text:
			mText = _x.mText;
			break;
		default: // Do nothing for null
			break;
		}
		return *this;
	}

	//------------------------------------------------------------------------------------------------------------------
	inline Json::~Json() {
		switch (mType)
		{
		case DataType::array:
			for(const auto& element : mArray)
				delete element;
			break;
		case DataType::object:
			for(const auto& element : mObject)
				delete element.second;
			break;
		default:
			break;
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	template<class T_>
	Json& Json::operator=(std::initializer_list<T_> _list) {
		clear();
		mType = DataType::array;
		mArray.reserve(_list.size());
		for(auto element : _list)
			mArray.push_back(new Json(element));
		return *this;
	}

	//------------------------------------------------------------------------------------------------------------------
	inline bool Json::isNull() const {
		return mType == DataType::null;
	}

	//------------------------------------------------------------------------------------------------------------------
	inline bool Json::isBool() const {
		return mType == DataType::boolean;
	}

	//------------------------------------------------------------------------------------------------------------------
	inline bool Json::isNumber() const {
		return mType == DataType::integer || mType == DataType::real;
	}

	//------------------------------------------------------------------------------------------------------------------
	inline bool Json::isString() const {
		return mType == DataType::text;
	}

	//------------------------------------------------------------------------------------------------------------------
	inline bool Json::isArray() const {
		return mType == DataType::array;
	}

	//------------------------------------------------------------------------------------------------------------------
	inline bool Json::isObject() const {
		return mType == DataType::object;
	}

	//------------------------------------------------------------------------------------------------------------------
	template<typename T_>
	void Json::push_back(const T_& _element) {
		if(mType == DataType::null)
			mType = DataType::array;
		assert(mType == DataType::array);
		mArray.push_back(new Json(_element));
	}

} // namespace cjson

#endif // _CJSON_JSON_INL_