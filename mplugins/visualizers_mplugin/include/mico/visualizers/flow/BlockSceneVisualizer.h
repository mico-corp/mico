//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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

#ifdef HAS_MICO_SLAM

#ifndef MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_BLOCKSCENEVISUALIZER_H_
#define MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_BLOCKSCENEVISUALIZER_H_


#include <flow/Block.h>

#include <mutex>
#include <deque>

#include <mico/visualizers/SceneVisualizer.h>

namespace mico{
        namespace visualizer{
    
        /// Mico block for visualizing 3D scenes, implemented using VTK PCL visualizer.
        /// @ingroup  mico_visualizer
        class BlockSceneVisualizer: public flow::Block{
        public:
            /// Get name of block
            virtual std::string name() const override { return "Scene Visualizer"; }

            BlockSceneVisualizer();
            ~BlockSceneVisualizer();



        bool configure(std::vector<flow::ConfigParameterDef> _params) override;
        /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;


        private:
            SceneVisualizer<pcl::PointXYZRGBNormal> sceneVisualizer_;

            void init();

        private:
            static bool sAlreadyExisting_;
            bool sBelonger_;

            std::thread spinnerThread_;
            bool run_ = true;
            bool idle_ = true;
            bool hasBeenInitialized_ = false;


            std::deque<Dataframe<pcl::PointXYZRGBNormal>::Ptr> queueDfs_;
            std::mutex queueDfGuard_;
            
    #ifdef HAS_MICO_DNN
            std::deque<std::vector<std::shared_ptr<dnn::Entity<pcl::PointXYZRGBNormal>>>> queueEntities_;
            std::mutex queueEntitiesGuard_;
    #endif
            bool hasPose = false;
            Eigen::Matrix4f lastPose_;
            std::mutex poseGuard_;

            // Parameters
            float voxelSize_ = -1;
            bool useOctree = false;
            bool octreeDepth = 4;
        };
    }
}

#endif

#endif