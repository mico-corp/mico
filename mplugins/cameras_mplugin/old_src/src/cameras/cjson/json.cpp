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
#include <mico/cameras/cjson/json.h>
#include <mico/cameras/cjson/parser.h>
#include <mico/cameras/cjson/serializer.h>
#include <cassert>
#include <new> // Placement new
#include <sstream>

namespace cjson {

	//------------------------------------------------------------------------------------------------------------------
	bool Json::parse(const char* _code) {
		setNull();
		Parser p(_code);
		if(!p.parse(*this)) {
			setNull();
			return false;
		}
		return true;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Json::parse(std::istream& _is) {
		setNull();
		Parser p(_is);
		if(!p.parse(*this)) {
			setNull();
			return false;
		}
		return true;
	}

	//------------------------------------------------------------------------------------------------------------------
	std::string Json::serialize() const {
		std::stringstream sstream;
		if(serialize(sstream))
			return sstream.str();
		else // Serialization went wrong.
			return "";
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Json::serialize(std::ostream& _dst) const {
		Serializer s;
		return s.serialize(*this, _dst);
	}

	//------------------------------------------------------------------------------------------------------------------
	void Json::setNull() {
		clear(); // Clear internal elements if necessary (must be done before changing type)
		mType = DataType::null; // Reset type
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::Json(bool _b)
		: mType(DataType::boolean)
	{
		mNumber.b = _b;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::Json(int _i)
		: mType(DataType::integer)
	{
		mNumber.i = _i;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::Json(unsigned _u)
		: mType(DataType::integer)
	{
		mNumber.i = int(_u);
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::Json(float _f)
		: mType(DataType::real)
	{
		mNumber.f = _f;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::Json(const char* _s)
		: mType(DataType::text),mText(_s)
	{
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::Json(const std::string& _s)
		: mType(DataType::text), mText(_s)
	{
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::Json(std::string&& _s)
		: mType(DataType::text), mText(std::move(_s))
	{
	}

	//------------------------------------------------------------------------------------------------------------------
	Json& Json::operator=(bool _b) {
		clear();
		this->~Json();
		new(this)Json(_b);
		return *this;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json& Json::operator=(int _i) {
		clear();
		this->~Json();
		new(this)Json(_i);
		return *this;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json& Json::operator=(unsigned _u) {
		return this->operator=(int(_u));
	}

	//------------------------------------------------------------------------------------------------------------------
	Json& Json::operator=(float _f) {
		clear();
		this->~Json();
		new(this)Json(_f);
		return *this;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json& Json::operator=(const char* _s) {
		clear();
		this->~Json();
		new(this)Json(_s);
		return *this;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json& Json::operator=(const std::string& _s) {
		clear();
		this->~Json();
		new(this)Json(_s);
		return *this;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json& Json::operator=(std::string&& _s) {
		clear();
		this->~Json();
		new(this)Json(std::move(_s));
		return *this;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Json::operator==(const Json& _x) const {
		if(mType != _x.mType)
			return false;
		switch (mType)
		{
		case DataType::null:
			return true;
		case DataType::text:
			return mText == _x.mText;
		case DataType::array:
			if(size() != _x.size())
				return false;
			for(size_t i = 0; i < size(); ++i) {
				if(!(*mArray[i] == *_x.mArray[i]))
					return false;
			}
			return true;
		case DataType::object:
			if(size() != _x.size())
				return false;
			for(const auto& myElement : mObject) {
				const std::string& key = myElement.first;
				if(!_x.contains(key))
					return false;
				if(!(_x[key] == *myElement.second))
					return false;
			}
			return true;
		default:
			return mNumber.i == _x.mNumber.i;
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Json::operator==(bool _b) const {
		assert(mType == DataType::boolean || mType == DataType::integer);
		return mNumber.b == _b;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Json::operator==(int _i) const {
		assert(mType == DataType::integer);
		return mNumber.i == _i;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Json::operator==(unsigned _u) const {
		assert(mType == DataType::integer);
		return unsigned(mNumber.i) == _u;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Json::operator==(float _f) const {
		assert(mType == DataType::real);
		return mNumber.f == _f;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Json::operator==(const char* _s) const {
		assert(mType == DataType::text);
		return mText == _s;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Json::operator==(const std::string& _s) const {
		assert(mType == DataType::text);
		return mText == _s;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::operator bool() const {
		assert(mType == DataType::boolean || mType == DataType::integer);
		return mNumber.b;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::operator int() const {
		assert(mType == DataType::integer);
		return mNumber.i;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::operator float() const {
		assert(mType == DataType::real);
		return mNumber.f;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::operator std::string() const {
		assert(mType == DataType::text);
		return mText;
	}

	//------------------------------------------------------------------------------------------------------------------
	const Json& Json::operator()(size_t _n) const {
		assert(mType == DataType::array);
		return *mArray[_n];
	}

	//------------------------------------------------------------------------------------------------------------------
	Json& Json::operator()(size_t _n) {
		assert(mType == DataType::array);
		return *mArray[_n];
	}
	
	//------------------------------------------------------------------------------------------------------------------
	const Json& Json::operator[](const char* _key) const {
		assert(mType == DataType::object);
		return *mObject.find(_key)->second;
	}
	
	//------------------------------------------------------------------------------------------------------------------
	Json& Json::operator[](const char* _key) {
		if(mType != DataType::object) {
			clear();
			mType = DataType::object;
		}
		Json*& objRef = mObject[_key]; // Pointer reference so we can point it to a new object
		if(objRef == nullptr)
			objRef = new Json();
		return *objRef;
	}
	
	//------------------------------------------------------------------------------------------------------------------
	const Json& Json::operator[](const std::string& _key) const {
		assert(mType == DataType::object);
		return *mObject.find(_key)->second;
	}
	
	//------------------------------------------------------------------------------------------------------------------
	Json& Json::operator[](const std::string& _key) {
		if(mType != DataType::object) {
			clear();
			mType = DataType::object;
		}
		Json*& objRef = mObject[_key];
		if(objRef == nullptr)
			objRef = new Json();
		return *objRef;
	}

	//------------------------------------------------------------------------------------------------------------------
	size_t Json::size() const {
		assert(mType == DataType::array || mType == DataType::object || mType == DataType::text);
		switch (mType)
		{
		case cjson::Json::DataType::text:
			return mText.size();
		case cjson::Json::DataType::array:
			return mArray.size();
		case cjson::Json::DataType::object:
			return mObject.size();
		default:
			assert(false);
			return size_t(-1);
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Json::contains(const std::string& _key) const {
		assert(mType == DataType::object);
		return mObject.find(_key) != mObject.end();
	}

	//------------------------------------------------------------------------------------------------------------------
	void Json::clear() {
		// Clear internal elements if necessary
		switch(mType) {
		case DataType::array:
			for(const auto& element : mArray)
				delete element;
			mArray.clear();
			break;
		case DataType::object:
			for(const auto& element : mObject)
				delete element.second;
			mObject.clear();
			break;
		default:
			break;
		}
	}

	//----------------------------------------------------------------------------------------------------------------------
	Json::Dictionary Json::mapDeepCopy(const Json::Dictionary &_map) {
		Dictionary copy;
		assert(isObject());

		for (auto element : _map) {
			copy[element.first] = new Json(*element.second);
		}
		return copy;
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::const_iterator Json::begin() const{
		if (isArray())
			return const_iterator(mArray.begin());
		else
			return const_iterator(mObject.begin());
	}
	
	//------------------------------------------------------------------------------------------------------------------
	Json::iterator Json::begin(){
		if (isArray())
			return iterator(mArray.begin());
		else
			return iterator(mObject.begin());
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::const_iterator Json::end() const{
		if (isArray())
			return const_iterator(mArray.end());
		else
			return const_iterator(mObject.end());
	}

	//------------------------------------------------------------------------------------------------------------------
	Json::iterator Json::end(){
		if (isArray())
			return iterator(mArray.end());
		else
			return iterator(mObject.end());
	}
	
	//------------------------------------------------------------------------------------------------------------------
}	// namespace cjson
