//---------------------------------------------------------------------------------------------------------------------
//  FLOW
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


#ifndef FLOW_OUTPIPE_H_
#define FLOW_OUTPIPE_H_

#include <flow/Export.h>

#include <vector>
#include <cstdlib>

#include <boost/any.hpp>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <iostream>
#include <functional>
#include <string>

#include <mutex>

#include <map>

namespace flow{
    class Policy;
    
    
    /// Base class of flow that represents an output stream of data.
    /// @ingroup  flow
    class FLOW_DECL Outpipe{
        public:
            /// Create an output pipe with a given name and type.
            Outpipe(std::string _tag, std::string _type);

            /// Create an output pipe with a given name and type.
            template<typename T_>
            explicit Outpipe(std::string _tag){
                type_ = typeid(T_).name();
                tag_ = _tag;
            }

            /// Associate an input stream to the pipe to be updated when the pipe flushes data.
            bool registerPolicy(Policy * _pol, std::string _policyTag);

            /// Deassociate an input stream.
            void unregisterPolicy(Policy* _pol);

            /// Get number of registered inputs
            int registrations();

            /// Flush data through the pipe
            void flush(boost::any _data);

            /// Get name of the pipe
            std::string tag() const;

            /// Get type of the pipe
            std::string type() const;
            

        protected:
            std::mutex policiesGuard;
            std::vector<Policy*> registeredPolicies_;
            std::map<Policy*, std::string> tagTranslators_;
            std::string tag_, type_;
            
    };
}



#endif