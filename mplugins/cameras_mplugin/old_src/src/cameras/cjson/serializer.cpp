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
#include <mico/cameras/cjson/serializer.h>
#include <mico/cameras/cjson/json.h>

#include <cassert>

using namespace std;

namespace cjson {
	//------------------------------------------------------------------------------------------------------------------
	bool Serializer::serialize(const Json& _j, ostream& _dst) {
		return push(_j, _dst);
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Serializer::push(const Json& _j, ostream& _oStream, size_t _tab, bool _skipFirstRowTab) {
		if(!_skipFirstRowTab)
			tabify(_oStream, _tab);
		switch (_j.mType)
		{
		case Json::DataType::null:
			_oStream << "null";
			return true;
		case Json::DataType::boolean:
			return push(_j.mNumber.b, _oStream);
		case Json::DataType::integer:
			_oStream << _j.mNumber.i;
			return true;
		case Json::DataType::real:
			_oStream << _j.mNumber.f;
			return true;
		case Json::DataType::text:
			_oStream << '\"' << _j.mText << '\"';
			return true;
		case Json::DataType::array:
			return push(_j.mArray, _oStream, _tab);
		case Json::DataType::object:
			return push(_j.mObject, _oStream, _tab);
		default:
			return false; // Error data type
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Serializer::push(bool _b, ostream& _oStream) {
		_oStream << (_b? "true" : "false");
		return true;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Serializer::push(const Json::Array& _array, ostream& _oStream, size_t _tab) {
		_oStream << "[\n"; // Open braces
		// Push elements
		for(size_t i = 0; i < _array.size(); ++i) {
			if(!push(*_array[i], _oStream, _tab+1))
				return false; // Error processing element
			if(i != _array.size()-1) // All elements but the last one
				_oStream << ',';
			_oStream << '\n';
		}
		// Close braces
		tabify(_oStream, _tab);
		_oStream << ']';
		return true;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Serializer::push(const Json::Dictionary& _obj, ostream& _oStream, size_t _tab) {
		_oStream << "{\n"; // Open braces
		// Push elements
		size_t i = 0;
		for(const auto& element : _obj) {
			tabify(_oStream, _tab+1);
			_oStream << "\"" << element.first << "\": "; // Key
			if(!push(*element.second, _oStream, _tab+1, true)) // Value
				return false; // Error processing element
			if(i < _obj.size()-1) // All elements but the last one
				_oStream << ',';
			_oStream << '\n';
			++i;
		}
		// Close braces
		tabify(_oStream, _tab);
		_oStream << '}';
		return true;
	}

	//------------------------------------------------------------------------------------------------------------------
	void Serializer::tabify(ostream& _oStream, size_t _tab) {
		for(size_t i = 0; i < _tab; ++i)
			_oStream << '\t';
	}

}	// namespace cjson
