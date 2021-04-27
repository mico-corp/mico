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
#include <cassert>

namespace cjson{
	//------------------------------------------------------------------------------------------------------------------
	template<class Type_>
	Iterator<Type_>::Iterator(){

	}
	//------------------------------------------------------------------------------------------------------------------
	template<class Type_>
	Iterator<Type_>::Iterator(typename Trait::mArrayIteratorType _iterator){
		mArrayIterator = _iterator;
		mIsArray = true;
	}

	//------------------------------------------------------------------------------------------------------------------
	template<class Type_>
	Iterator<Type_>::Iterator(typename Trait::mObjIteratorType _iterator){
		mObjectIterator = _iterator;
		mIsArray = false;
	}

	//------------------------------------------------------------------------------------------------------------------
	template<class Type_>
	Type_& Iterator<Type_>::operator*(){
		if (mIsArray){
			return **mArrayIterator;
		}
		else{
			return *((*mObjectIterator).second);
		}

	}

	//------------------------------------------------------------------------------------------------------------------
	template<class Type_>
	Iterator<Type_>& Iterator<Type_>::operator++(){
		if (mIsArray){
			mArrayIterator++;
		}
		else{
			mObjectIterator++;
		}
		return *this;
	}

	//------------------------------------------------------------------------------------------------------------------
	template<class Type_>
	Iterator<Type_> Iterator<Type_>::operator++(int){
		Iterator<Type_> res(*this);
		++(*this);
		return res;
	}

	//------------------------------------------------------------------------------------------------------------------
	template<class Type_>
	Type_* Iterator<Type_>::operator->(){
		if (mIsArray){
			return *mArrayIterator;
		}
		else{
			return (*mObjectIterator).second;
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	template<class Type_>
	bool Iterator<Type_>::operator==(Iterator<Type_> _iter){
		if (mIsArray){
			return mArrayIterator == _iter.mArrayIterator ? true : false;
		}
		else{
			return mObjectIterator == _iter.mObjectIterator ? true : false;
		}
	}
	
	//------------------------------------------------------------------------------------------------------------------
	template<class Type_>
	bool Iterator<Type_>::operator!=(Iterator<Type_> _iter){
		if (mIsArray){
			return mArrayIterator != _iter.mArrayIterator ? true : false;
		}
		else{
			return mObjectIterator != _iter.mObjectIterator ? true : false;
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	template<class Type_>
	const std::string& Iterator<Type_>::key(){
		assert(!mIsArray);

		return (*mObjectIterator).first;
	}

}	//	 namespace cjson;