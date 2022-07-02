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



#ifndef MICO_AR_FLOW_BLOCKARVIEWER_H_
#define MICO_AR_FLOW_BLOCKARVIEWER_H_

#include <flow/Block.h>
#include <mico/ar/gl_helpers/VisualizerGlWidget.h>

namespace mico{
    namespace ar {
        /// Mico block instantiate a 3D visualizer for AR blocks.
        /// @ingroup  mico_ar
        ///
        /// @image html blocks/ar/ar_block_ar_viewer.png width=480px
        ///
        /// __Inputs__:
        ///     * coordinates: 4x4 matrix of type Eigen::Matrix4f representing the position of the camera 
        ///     * image: image of type cv::Mat showing cameras PoV
        ///
        class BlockArViewer:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Block AR viewer";}        
            
            /// Base constructor that initializes the pipes
            BlockArViewer();

            /// Base destructor that initializes the pipes
            ~BlockArViewer();

            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir() / "ar" / "block_ar_viewer.svg").string();
            }

            /// Returns a brief description of the block
            std::string description() const override {return    "Block AR viewer"
                                                                "   - \n";};

            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

        private:
            void policyCallback(Eigen::Matrix4f _coordinates, cv::Mat _image);

        private:
            VisualizerGlWidget* widget_ = nullptr;
        };

    }

}



#endif