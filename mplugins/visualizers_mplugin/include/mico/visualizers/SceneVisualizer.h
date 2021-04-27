#ifdef HAS_MICO_SLAM

#ifndef SLAMMARKI_SCENEVISUALIZER_H_
#define SLAMMARKI_SCENEVISUALIZER_H_


#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <thread>

#include <mico/slam/Dataframe.h>
// #include <mico/slam/cjson/json.h>

#include <pcl/octree/octree_pointcloud_occupancy.h>

#include <pcl/filters/voxel_grid.h>

#ifdef HAS_MICO_DNN
    #include <mico/dnn/map3d/Entity.h>
#endif

namespace mico {
    template <typename PointType_>
    class SceneVisualizer
    {
    public:
        SceneVisualizer():mOctreeVis(0.01){};
        ~SceneVisualizer();

        /// Initializes SceneVisualizer parameters
        bool init(/*cjson::Json _configFile*/);//, DatabaseCF<PointType_> *_database);o

        void close();

        void cleanAll(){
            if(!mViewer)
                return;
                
            mViewer->removeAllPointClouds();
            mViewer->removeAllCoordinateSystems();
            mViewer->removeAllShapes();
        }


        void drawDataframe(std::shared_ptr<mico::Dataframe<PointType_>> &_df, bool _drawPoints = false);
        void updateDataframe(int _dfd, const Eigen::Matrix4f &_newPose);
        void drawWords(std::map<int, std::shared_ptr<Word<PointType_>>> _words);

        // Check if dataframesframes have been optimized to updated them
        void checkAndRedrawCf();

#ifdef HAS_MICO_DNN
        void drawEntity(std::vector<std::shared_ptr<dnn::Entity<PointType_>>> _entity, bool _drawPoints, bool _drawCube, float _opacity);
#endif
        // Draw every word optimized
        bool draw3DMatches(pcl::PointCloud<PointType_> _pc1, pcl::PointCloud<PointType_> _pc2);
        
        bool updateCurrentPose(const Eigen::Matrix4f &_pose);

        void pause();
        void spinOnce();
        void keycallback(const pcl::visualization::KeyboardEvent &_event, void *_data);
        void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void);
        void pointPickedCallback(const pcl::visualization::PointPickingEvent &event,void*viewer_void);

        typedef std::function<void(const pcl::visualization::KeyboardEvent &, void *)> CustomCallbackType;
        void addCustomKeyCallback(CustomCallbackType _callback);

    private:
        void insertNodeCovisibility(const Eigen::Vector3f &_position);
        void updateNodeCovisibility(int _id, const Eigen::Vector3f &_position);
        void addCovisibility(int _id,const std::vector<typename Dataframe<PointType_>::Ptr> &_others);
    
    private:
        std::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;

        float x,y,z;
        typename pcl::octree::OctreePointCloudOccupancy<PointType_> mOctreeVis;
        typedef typename pcl::octree::OctreePointCloudOccupancy<PointType_>::Iterator OctreeIterator;

        bool mUseOctree = false;
        int mOctreeDepth = 1;

        std::map<int, std::shared_ptr<Dataframe<PointType_>>> mDataframes;

        typename pcl::PointCloud<PointType_>::Ptr wordCloud = typename pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
        
        bool mDenseVisualization=true;

        bool mPause = false;

        std::vector<CustomCallbackType> mCustomCallbacks;

        vtkSmartPointer<vtkPolyData> mCovisibilityGraph;
        vtkSmartPointer<vtkUnsignedCharArray> mCovisibilityNodeColors;
        vtkSmartPointer<vtkPoints> mCovisibilityNodes;

        std::map<int,bool> mExistingDf;
        std::map<int,bool> mExistingEntity;
        std::map<int,bool> mExistingEntityCameras;
        std::map<int,int> mNodeCovisibilityCheckSum;

        bool mUseVoxel = false;
        pcl::VoxelGrid<PointType_> mVoxeler;
        
    };
} // namespace mico

#include "SceneVisualizer.inl"

#endif //HAS_MICO_SLAM

#endif // SLAMMARKI_VISUALIZER_H_