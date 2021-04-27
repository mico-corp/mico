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



#ifndef _CJSON_JSONITERATORS_H_
#define _CJSON_JSONITERATORS_H_

namespace cjson{
	// Traits
	template<class Type_>
	struct IteratorTrait{
		typedef typename Type_::Array::iterator mArrayIteratorType;
		typedef typename Type_::Dictionary::iterator mObjIteratorType;
	};

	template<class Type_> 
	struct IteratorTrait<const Type_>{
		typedef typename Type_::Array::const_iterator mArrayIteratorType;
		typedef typename Type_::Dictionary::const_iterator mObjIteratorType;
	};

	// Template
	template<class Type_>
	class Iterator{
	public:
		typedef IteratorTrait<Type_>	Trait;

		Iterator();

		Iterator(typename Trait::mArrayIteratorType _iterator);
		Iterator(typename Trait::mObjIteratorType _iterator);

		Type_&		operator*() const;
		Type_&		operator*();

		Iterator<Type_>&			operator++();
		Iterator<Type_>				operator++(int);
		Type_*		operator->();

		bool	operator==(Iterator<Type_> _iter);
		bool	operator!=(Iterator<Type_> _iter);

		const std::string&		key();

	private:
		bool mIsArray;

		typename Trait::mArrayIteratorType	mArrayIterator;
		typename Trait::mObjIteratorType	mObjectIterator;
	};
}

#include "JsonIterator.inl"

#endif	//	_CJSON_JSONITERATORS_H_