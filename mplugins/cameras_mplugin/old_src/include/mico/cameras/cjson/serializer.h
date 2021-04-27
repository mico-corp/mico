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
#ifndef _CJSON_SERIALIZER_H_
#define _CJSON_SERIALIZER_H_

#include <string>
#include <iostream>
#include "json.h"

namespace cjson {

	class Serializer {
	public:
		/// Serialize the contents of a json into a std::ostream
		/// It translates the binary Json _j into standard text format.
		///\return true on success, 0 on serialization error.
		bool serialize(const Json& _j, std::ostream& _dst);

	private:
		bool push(const Json&, std::ostream& _dst, size_t _tab = 0, bool _skipFirstRowTab = false);
		bool push(bool, std::ostream& _dst);
		bool push(const Json::Array&, std::ostream& _dst, size_t _tab = 0);
		bool push(const Json::Dictionary&, std::ostream& _dst, size_t _tab = 0);

		void tabify(std::ostream& _dst, size_t _tab);
	};

}	// namespace cjson

#endif // _CJSON_SERIALIZER_H_