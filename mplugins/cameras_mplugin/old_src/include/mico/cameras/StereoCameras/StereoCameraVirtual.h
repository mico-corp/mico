//---------------------------------------------------------------------------------------------------------------------
//  Cameras wrapper MICO plugin
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



#ifndef MICO_BASE_VISION_STEREOCAMERAS_STEREOCAMERAVIRTUAL_H_
#define MICO_BASE_VISION_STEREOCAMERAS_STEREOCAMERAVIRTUAL_H_


#include <mico/cameras/StereoCamera.h>
namespace mico {
	/// Util class to simulate an stereo camera using a dataset.
	class StereoCameraVirtual :public StereoCamera {
	public:		// Public interface
		/// \brief Initialize the camera using a config file.
		/// Config file must have following structure.
		///
		/// \code
		///     {
        ///         "input":
        ///             {
        ///                 "left":"/dir/to/file/template %d.jpg",          // Path template to a set of left images files.
        ///                 "right":"/dir/to/file/template %d.jpg",         // Path template to a set of right images files.
        ///                 "depth":"/dir/to/file/template %d.jpg",         // Path template to a set of depth images files.
        ///                 "pointCloud":"/dir/to/file/template %d.jpg"     // Path template to a set of point cloud files.
        ///             },
		///	        "loop_dataset":true|false,                      // re-start dataset when end is reached 
        ///         "calibFile":"/dir/to/calib/file.xml"            // Path to the calibration file (optional).
		///     }
		/// \endcode
		///
		/// \param _filePath: path to the file.
		bool init(const cjson::Json &_json = "");

		/// \brief Get RGB pair of images
		/// \param _left: referente to a container for the left image.
		/// \param _right: referente to a container for the right image.
		bool rgb(cv::Mat &_left, cv::Mat &_right);

		/// \brief Obtaing depth image.
		/// \param _depth: referente to a container for the depth image.
		bool depth(cv::Mat &_depth);

		/// \brief Grab current data from camera to make sure that is synchronized
		bool grab();

		/// \brief Get point cloud.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud);

        /// \brief Get colorized point cloud
        /// \param _cloud: reference to a container for the point cloud.
        bool cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud);

		/// \brief [DUMMY] Override of cloud method of StereoCamera.
		bool cloud(pcl::PointCloud<pcl::PointNormal> &_cloud);

		/// \brief [DUMMY] Override of cloud method of StereoCamera.
		bool cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud);

        /// \brief templatized method to define the interface for retrieving a new point cloud from the camera with
        /// spatial, surface normals and RGB information (custom types out of PCL).
        /// \param _cloud: reference to a container for the point cloud.
        template<typename PointType_>
        bool cloud(pcl::PointCloud<PointType_> &_cloud);

        /// \brief get the calibration matrices of the left camera in opencv format. Matrices are CV_32F.
        virtual bool leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients);

        /// \brief get the calibration matrices of the right camera in opencv format. Matrices are CV_32F.
        virtual bool rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients);

        /// \brief get the extrinsic matrices, i.e., transformation from left to right camera. Matrices are CV_32F.
        virtual bool extrinsic(cv::Mat &_rotation, cv::Mat &_translation);

        /// \brief get the extrinsic matrices, i.e., transformation from left to right camera.
        virtual bool extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation);

        /// \brief get disparty-to-depth parameter typical from RGB-D devices. Not all devices have this variable.
        virtual bool disparityToDepthParam(double &_dispToDepth);

        bool colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point);

    private:	// Private methods
        void depthToPointcloud(cv::Mat &_depth, pcl::PointCloud<pcl::PointXYZ> &_cloud);
        void depthToPointcloud(cv::Mat &_depth, pcl::PointCloud<pcl::PointXYZRGB> &_cloud);
	private:	// Private members
		unsigned mFrameCounter = 1;
		unsigned mStepIdx = 1;

        cv::Mat mLeft, mRight, mDepth;
        pcl::PointCloud<pcl::PointXYZ> mCloudXYZ;
        pcl::PointCloud<pcl::PointXYZRGB> mCloudXYZRGB;
        pcl::PointCloud<pcl::PointXYZRGBA> mCloudXYZRGBA;
        pcl::PointCloud<pcl::PointXYZRGBNormal> mCloudXYZRGBNormal;

		std::string mLeftImageFilePathTemplate;
		std::string mRightImageFilePathTemplate;
		std::string mDepthImageFilePathTemplate;
		std::string mPointCloudFilePathTemplate;

        bool mHasCalibration = false;
        cv::Mat mMatrixLeft, mDistCoefLeft, mMatrixRight, mDistCoefRight, mRot, mTrans;

        double mDispToDepth;
        bool mLoopDataset = false;

	};
}	//	namespace mico 

#include <mico/cameras/StereoCameras//StereoCameraVirtual.inl>

#endif		// RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAVIRTUAL_H_
