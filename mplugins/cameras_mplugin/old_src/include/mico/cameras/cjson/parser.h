//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2015 Carmelo J. Fernández-Agüera Tortosa
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
#ifndef _CJSON_PARSER_H_
#define _CJSON_PARSER_H_

#include <string>

namespace cjson {

	class Json;

	///\ class Parser
	///\ brief Parse strings of characters into Json objects
	class Parser {
	public:
		///\param _s The parser will read from this stream every time it is requested to parse a Json
		/// It must provide valid, well formed, serialized Jsons.
		Parser(std::istream& _s);
		///\param _s The parser will read from this stream every time it is requested to parse a Json
		/// It must provide valid, well formed, serialized Jsons.
		Parser(const char* _s);
		~Parser();
		/// Fill in the Json with content from the parser's stream.
		///\ param _dst a Json object into which parse results will be stored
		///\ return \c true if able to retrieve content from the current stream and parse from it, \c false on error
		bool parse(Json& _dst);

		/// Replace the internal stream used to parse Jsons from.
		///\param _new The new stream to read from.
		///\return a reference to the old stream.
		std::istream& setStream(std::istream& _new);
		std::istream& getStream() const;

	private:
		bool parseNull(Json& _dst);
		bool parseFalse(Json& _dst);
		bool parseTrue(Json& _dst);
		bool parseNumber(Json& _dst);
		bool parseString(Json& _dst);
		bool parseArray(Json& _dst);
		bool parseObject(Json& _dst);
		bool parseInt(const std::string& _num, Json& _dst);
		bool parseFloat(const std::string& _num, Json& _dst);
		bool parseObjectEntry(std::string& _key, Json& _value);

		void skipWhiteSpace();

		std::istream* mIn;
		bool mOwnStream; ///< Wether we have strong ownership of the input stream.

	private:
		const std::string cSpacers = " \t\n\r";
		const std::string cDigits = "0123456789";
	};

}	// namespace cjson

#endif // _CJSON_PARSER_H_