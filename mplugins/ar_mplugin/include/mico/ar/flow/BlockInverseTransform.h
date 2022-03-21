//---------------------------------------------------------------------------------------------------------------------
//  AR MICO plugin
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



#ifndef MICO_AR_FLOW_BLOCKINVERSETRANSFORM_H_
#define MICO_AR_FLOW_BLOCKINVERSETRANSFORM_H_

#include <flow/Block.h>
#include <opencv2/opencv.hpp>

namespace cv{
    namespace aruco {
        class Dictionary;
    }
}

namespace mico{
    namespace ar {
        /// Mico block that computes the inverse transformation of a given 4x4 matrix that typically
        /// represents a 3D coordinate in the space.
        /// @ingroup  mico_ar
        class BlockInverseTransform:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Inverse Transform";}        
            
            /// Base constructor that initializes the pipes
            BlockInverseTransform();

            /// Retreive icon of block    
            QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "ar" / "block_inverse_transform.svg").string().c_str());
            }

            /// Returns a brief description of the block
            std::string description() const override {return    "Inverse Transform"
                                                                "   - \n";};

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };
        private:
            bool idle_ = true;
        };

    }

}



#endif