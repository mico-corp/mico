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

#include <mico/visualizers/flow/BlockSceneVisualizer.h>
#include <flow/Policy.h>
#include <flow/Outpipe.h>


#include <mico/slam/Dataframe.h>

#ifdef HAS_MICO_DNN
    #include <mico/dnn/map3d/Entity.h>
#endif

#include <Eigen/Eigen>


namespace mico{
    namespace visualizer{
        // Static member initialiation
        bool BlockSceneVisualizer::sAlreadyExisting_ = false;

        BlockSceneVisualizer::BlockSceneVisualizer(){
            if(sAlreadyExisting_)
                return;

            sAlreadyExisting_ = true;
            sBelonger_ = true;


            createPolicy({
                {"Camera Pose", "mat44"},
                {"Dataframe", "dataframe"},
                {"Entities", "v_entity"},
                {"Update Entities", "v_entity"}
            });

            registerCallback({ "Camera Pose" }, 
                                    [&](flow::DataFlow  _data){
                                        auto pose = _data.get<Eigen::Matrix4f>("Camera Pose"); 
                                        poseGuard_.lock();
                                        lastPose_ = pose;
                                        poseGuard_.unlock();
                                        hasPose = true;
                                    }
                                );

            registerCallback({ "Dataframe" }, 
                                    [&](flow::DataFlow  _data){
                                        auto df = _data.get<Dataframe<pcl::PointXYZRGBNormal>::Ptr>("Dataframe"); 
                                        queueDfGuard_.lock();
                                        queueDfs_.push_back(df);
                                        queueDfGuard_.unlock();
                                        idle_ = true;
                                    }
                                );

        #ifdef HAS_MICO_DNN
            registerCallback({"Entities" }, 
                                    [&](flow::DataFlow  _data){
                                        auto entities = _data.get<std::vector<std::shared_ptr<dnn::Entity<pcl::PointXYZRGBNormal>>>>("Entities"); 
                                        queueEntitiesGuard_.lock();
                                        queueEntities_.push_back(entities);
                                        queueEntitiesGuard_.unlock();
                                        idle_ = true;
                                    }
                                );

            registerCallback({"Update Entities" }, 
                                    [&](flow::DataFlow  _data){
                                        auto entities = _data.get<std::vector<std::shared_ptr<dnn::Entity<pcl::PointXYZRGBNormal>>>>("Update Entities"); 
                                        queueEntitiesGuard_.lock();
                                        queueEntities_.push_back(entities);
                                        queueEntitiesGuard_.unlock();
                                        idle_ = true;
                                    }
                                );
        #endif
            
        }

        BlockSceneVisualizer::~BlockSceneVisualizer(){
            if(sBelonger_){
                run_ = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if(spinnerThread_.joinable())
                    spinnerThread_.join();
                sAlreadyExisting_ = false;
            }
        }


        bool BlockSceneVisualizer::configure(std::vector<flow::ConfigParameterDef> _params) {
            {
                std::istringstream istr(_params["voxel_size"]);
                istr >> voxelSize_;
            }
            {
                std::istringstream istr(_params["use_octree"]);
                istr >> useOctree;
            }
            {
                std::istringstream istr(_params["octree_depth"]);
                istr >> octreeDepth;
            }

            if(!hasBeenInitialized_){
                init();
            }

            return true;
        }

        std::vector<flow::ConfigParameterDef>  BlockSceneVisualizer::parameters() {
            return {    {"voxel_size", flow::ConfigParameterDef::eParameterType::DECIMAL}, 
                        {"use_octree", flow::ConfigParameterDef::eParameterType::BOOLEAN},
                        {"octree_depth", flow::ConfigParameterDef::eParameterType::INTEGER}
                        };
        }

        void BlockSceneVisualizer::init(){
            spinnerThread_ = std::thread([this]{
                //cjson::Json configFile;
                /*configFile["enable"] = true;
                if(voxelSize_ > 0)
                    configFile["voxel_size"] = voxelSize_;
                if(useOctree){
                    configFile["use_octree"] = useOctree;
                    configFile["octree_depth"] = octreeDepth;
                }*/

                sceneVisualizer_.init(/*configFile*/);

                while(run_){
                    if(hasPose){
                        // update last pose
                        poseGuard_.lock();
                        Eigen::Matrix4f pose = lastPose_;
                        poseGuard_.unlock();
                        sceneVisualizer_.updateCurrentPose(pose);
                    }

                    while(queueDfs_.size() > 0){
                        queueDfGuard_.lock();
                        auto df = queueDfs_.front();
                        queueDfs_.pop_front();
                        queueDfGuard_.unlock();
                        sceneVisualizer_.drawDataframe(df);
                    }

                #ifdef HAS_MICO_DNN
                    while(queueEntities_.size() > 0){
                        queueEntitiesGuard_.lock();
                        auto e = queueEntities_.front();
                        queueEntities_.pop_front();
                        queueEntitiesGuard_.unlock();
                        sceneVisualizer_.drawEntity(e, true, true, 0.05);
                    }
                #endif
                    // Check optimizations.
                    sceneVisualizer_.checkAndRedrawCf();
                    
                    // Spin once.
                    sceneVisualizer_.spinOnce();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                // Self destroy
                sceneVisualizer_.close();
            });
        }
    }
}


#endif