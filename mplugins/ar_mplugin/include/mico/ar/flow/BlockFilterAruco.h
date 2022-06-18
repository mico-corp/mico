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



#ifndef MICO_AR_FLOW_BLOCKFILTERARUCO_H_
#define MICO_AR_FLOW_BLOCKFILTERARUCO_H_

#include <flow/Block.h>
#include <opencv2/opencv.hpp>


namespace mico{
    namespace ar {
        /// Mico block that Filters aruco detections by ID.
        /// @ingroup  mico_ar
        ///
        /// @image html blocks/ar/ar_block_filter_aruco_cs.png width=480px
        ///
        /// __Inputs__:
        ///     * all_coordinates: vector of 4x4 matrices of type Eigen::Matrix4f representing the coordinate frames of multiple aruco tags.
        ///
        /// __Outputs__:
        ///     * coordinates: 4x4 matrix of type Eigen::Matrix4f representing the coordinate frame of the aruco tag with selected ID.
        ///
        /// __parameters__:
        ///     * id: ID to filter aruco tag.
        ///
        class BlockFilterAruco:public flow::Block{
        public:
            /// Get name of block
            virtual std::string name() const override {return "Block Filter Aruco CS";}        
            
            /// Base constructor that initializes the pipes
            BlockFilterAruco();

            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;
            
            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Block Filter Aruco CS"
                                                                "   - \n";};


        private:
            int id_ = 0;
        };
    }


}



#endif