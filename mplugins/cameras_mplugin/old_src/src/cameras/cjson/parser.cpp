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
#include <mico/cameras/cjson/parser.h>
#include <cstring>
#include <sstream>
#include <string>
#include <mico/cameras/cjson/json.h>

#if defined(_WIN32) && defined(_DEBUG) // Trace memory leaks
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#endif

#if defined(_WIN32) && defined(_DEBUG) // Trace memory leaks
#define DEBUG_CLIENTBLOCK new( _CLIENT_BLOCK, __FILE__, __LINE__)
#define new DEBUG_CLIENTBLOCK
#endif

namespace cjson {

	//------------------------------------------------------------------------------------------------------------------
	Parser::Parser(std::istream& _s)
		:mIn(&_s)
		,mOwnStream(false)
	{
		// Intentionally blank
	}

	//------------------------------------------------------------------------------------------------------------------
	Parser::Parser(const char* _s)
		:mOwnStream(true)
	{
		mIn = new std::stringstream(std::string(_s));
	}

	//------------------------------------------------------------------------------------------------------------------
	Parser::~Parser()
	{
		if(mOwnStream)
			delete mIn;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Parser::parse(Json& _dst)
	{
		skipWhiteSpace();
		char c = mIn->peek();
		switch (c)
		{
		case 'n': return parseNull(_dst);
		case 't': return parseTrue(_dst);
		case 'f': return parseFalse(_dst);
		case '\"': return parseString(_dst);
		case '[': return parseArray(_dst);
		case '{': return parseObject(_dst);
		default:
			// Is it a number?
			if(((c >= '0') && (c <= '9')) || (c == '+') || (c == '-'))
				return parseNumber(_dst);
			// Unsupported, return parsing error
			return false;
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Parser::parseNull(Json& _dst) {
		char buff[4];
		mIn->read(buff,4);
		return 0 == strncmp("null",buff,4);
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Parser::parseTrue(Json& _dst) {
		_dst.mType = Json::DataType::boolean;
		char buff[4];
		mIn->read(buff,4);
		_dst = true;
		return 0 == strncmp("true",buff,4);
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Parser::parseFalse(Json& _dst) {
		_dst.mType = Json::DataType::boolean;
		char buff[5];
		mIn->read(buff,5);
		_dst = false;
		return 0 == strncmp("false",buff,5);
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Parser::parseNumber(Json& _dst) {
		// Skip all digits
		std::string num;
		if(mIn->peek() == '+')
			mIn->ignore();
		if(mIn->peek() == '-')
			num += mIn->get();
		while(cDigits.find(mIn->peek()) != std::string::npos) {
			num += char(mIn->get());
		}
		int c = mIn->peek();
		// Either parse as a float or an int
		if(c == '.') {
			num += mIn->get();
			// Parse the rest of the number
			while(cDigits.find(mIn->peek()) != std::string::npos) {
				num += char(mIn->get());
			}
			if (mIn->peek() == 'f') {
				mIn->ignore();
			}
			
			return parseFloat(num, _dst);
		}
		else
			return parseInt(num, _dst);
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Parser::parseString(Json& _dst) {
		mIn->ignore(1); // Skip opening quotes
		bool escaped = false;
		std::ostringstream oss;
		// Read until the first unescaped quote
		for(int c = mIn->get(); (c != '"') || escaped; c = mIn->get()) {
			if(mIn->eof())
				return false; // Unterminated string
			if(escaped || c == '\\')
				escaped ^= 1; // Negate escape state
			else
				oss << (char)c;
		}
		_dst = oss.str(); // Do not include the quote we just read.
		return true;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Parser::parseArray(Json& _dst) {
		_dst.mType = Json::DataType::array;
		mIn->ignore(); // Skip [
		skipWhiteSpace();
		Json element;
		while(mIn->peek() != ']') {
			// Parse element
			if(!parse(element))
				return false;
			_dst.push_back(element);
			// Read upto the next element
			skipWhiteSpace();
			if(mIn->peek() == ',') {
				mIn->ignore();
				skipWhiteSpace();
			}
		}
		mIn->ignore(); // Skip ]
		return true;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Parser::parseObject(Json& _dst) {
		_dst.mType = Json::DataType::object;
		mIn->ignore(); // Skip {
		skipWhiteSpace();
		std::string key;
		while(mIn->peek() != '}') {
			Json value;
			// Parse element
			if(!parseObjectEntry(key,value))
				return false;
			_dst[key] = value;
			// Read upto the next element
			skipWhiteSpace();
			if(mIn->peek() == ',') {
				mIn->ignore();
				skipWhiteSpace();
			}
		}
		mIn->ignore(); // Skip }
		return true;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Parser::parseInt(const std::string& _num, Json& _dst) {
		_dst.mType = Json::DataType::integer;
		int i;
		std::stringstream(_num) >> i;
		_dst = i;
		return true;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Parser::parseFloat(const std::string& _num, Json& _dst) {
		_dst.mType = Json::DataType::real;
		float f;
		std::stringstream(_num) >> f;
		_dst = f;
		return true;
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Parser::parseObjectEntry(std::string& _oKey, Json& _dst) {
		skipWhiteSpace();
		if(mIn->peek() == '"'){
			Json key;
			if(!parseString(key)) // Key 
				return false;
			_oKey = std::string(key);
		}
		else { // Unquoted key
			_oKey = "";
			while (mIn->peek() != ':') {
				if(mIn->eof())
					return false;
				_oKey = _oKey + char(mIn->get());
				skipWhiteSpace();
			}
		}
		skipWhiteSpace();
		if(mIn->get() != ':')
			return false;
		parse(_dst); // Value
		return true;
	}

	//------------------------------------------------------------------------------------------------------------------
	void Parser::skipWhiteSpace() {
		while(cSpacers.find(mIn->peek()) != std::string::npos)
			mIn->ignore();
	}

}	// namespace cjson
