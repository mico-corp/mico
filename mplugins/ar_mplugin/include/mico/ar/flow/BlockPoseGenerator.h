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



#ifndef MICO_AR_FLOW_BLOCKPOSEGENERATOR_H_
#define MICO_AR_FLOW_BLOCKPOSEGENERATOR_H_

#include <flow/Block.h>

namespace mico{
    namespace ar {
        /// Mico block that generates a 3D pose in the space from given position and orientation
        /// parameters.
        /// @ingroup  mico_ar
        ///
        /// @image html blocks/ar/ar_block_filter_aruco_cs.png width=480px
        ///
        /// __Inputs__:
        ///     * x: component x of the translation
        ///     * y: component y of the translation
        ///     * z: component z of the translation
        ///     * roll: Rotation over the X axis
        ///     * pitch: Rotation over the Y axis
        ///     * yaw: Rotation over the Z axis
        ///
        /// __Outputs__:
        ///     * pose: 4x4 matrix of type Eigen::Matrix4f representing a 3D pose.
        ///
        /// __parameters__:
        ///     * absolute: 
        ///
        class BlockPoseGenerator:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Block Pose Generator";}        
            
            /// Base constructor that initializes the pipes
            BlockPoseGenerator();

            /// Retreive icon of block    
            QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "ar" / "block_transform_generator.svg").string().c_str());
            }

            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;
            
            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Generator"
                                                                "   - \n";};
        private:
            void policyCallback(float _x, float _y, float _z, float _roll, float _pitch, float _yaw);

        private:
            bool isAbsolute_ = true;
        };

    }

}



#endif