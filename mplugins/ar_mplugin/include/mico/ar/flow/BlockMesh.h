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



#ifndef MICO_AR_FLOW_BLOCKMESH_H_
#define MICO_AR_FLOW_BLOCKMESH_H_

#include <flow/Block.h>
#include <mico/ar/gl_helpers/VisualizerGlWidget.h>
#include <mico/ar/gl_helpers/Scene3d.h>
#include <mico/ar/gl_helpers/Mesh.h>

#include <Eigen/Eigen>

namespace mico{
    namespace ar {
        /// Mico block that loads an STL mesh into the AR viewer. Its pose in the space is fed
        /// by te input stream.
        /// @ingroup  mico_ar
        ///
        /// @image html blocks/ar/ar_block_mesh.png width=480px
        ///
        /// __Inputs__:
        ///     * coordinates: 4x4 matrix of type Eigen::Matrix4f representing the pose where to draw the mesh.
        ///
        /// __parameters__:
        ///     * mesh_path: path to an stl file to be displayed in the enabled Block AR Viewer.
        ///
        class BlockMesh:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Block Mesh";}        
            
            /// Base constructor that initializes the pipes
            BlockMesh();

            /// Retreive icon of block    
            QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "ar" / "block_mesh.svg").string().c_str());
            }

            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;
            
            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Bloc kMesh"
                                                                "   - \n";};

        private:
            void policyCallback(Eigen::Matrix4f _coordinates);
        private:
            Scene3d * scene_;
            std::shared_ptr<Mesh> mesh_;

        };

    }

}



#endif