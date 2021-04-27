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


#ifndef MICO_BASE_VISION_STEREOCAMERA_H_
#define MICO_BASE_VISION_STEREOCAMERA_H_

#include <mico/cameras/cjson/json.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>

#include <string>
#include <iostream>

namespace mico {
	/// \brief Abstract class of a stereo camera device. It defines the typical generic interfaces for devices that acquire 3d information
	/// as 3d scanners, RGB-D sensors, stereo cameras, etc.
	class StereoCamera {
	public:	// Static interface
        enum class eModel { Zed, ArtecEva, Virtual, Custom, Http, RealSense, Kinect, ROS, rosbag };
		
		/// Create a camara of the available types in @StereoCamera::eModel
		/// \param _type: type of camera to be created
		static StereoCamera *create(eModel _type);

		/// Create a camara of the available types {"virtual", "zed", "artec", "custom", "realsense", "kinect"}
		/// \param _type: type of camera to be created
		static StereoCamera *create(std::string _type);

	public:	// Public interface
		/// \brief Abstract method to define the interface for the initialization of the cameras.
		/// \param _filePath: path to the file.
		virtual bool init(const cjson::Json &_json = "") = 0;

		/// \brief Abstract method to define the interface for retrieving RGB information (Or pair of RGB images).
		/// \param _left: referente to a container for the left image.
		/// \param _right: referente to a container for the right image.
		virtual bool rgb(cv::Mat &_left, cv::Mat &_right) = 0;

		/// \brief Abstract method to define the interface for retrieving depth image.
		virtual bool depth(cv::Mat &_depth) = 0;

		/// \brief Abstract method to define the interface for retrieving a new point cloud from the camera with
		/// only spatial information.
		/// \param _cloud: reference to a container for the point cloud.
		virtual bool cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud) = 0;

		/// \brief Abstract method to define the interface for retrieving a new point cloud from the camera with 
		/// spatial and RGB information.
		/// \param _cloud: reference to a container for the point cloud.
		virtual bool cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud) = 0;

		/// \brief Abstract method to define the interface for retrieving a new point cloud from the camera with
		/// spatial information and surface normals
		/// \param _cloud: reference to a container for the point cloud.
		virtual bool cloud(pcl::PointCloud<pcl::PointNormal> &_cloud) = 0;

		/// \brief Abstract method to define the interface for retrieving a new point cloud from the camera with
		/// spatial, surface normals and RGB information.
		/// \param _cloud: reference to a container for the point cloud.
		virtual bool cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud) = 0;

        /// \brief templatized method to define the interface for retrieving a new point cloud from the camera with
        /// spatial, surface normals and RGB information (custom types out of PCL).
        /// \param _cloud: reference to a container for the point cloud.
        template<typename PointType_>
        bool cloud(pcl::PointCloud<PointType_> &_cloud);

		/// \brief Abstract method to define the interface for grabing the current data from camera to make sure that is synchronized
		virtual bool grab() = 0;

		/// \brief Changes the IR laser power level, 0 to turn off (only available for some active stereo devices)
        virtual bool laserPower(double power_level){return false;}

        /// \brief get the calibration matrices of the left camera in opencv format.  Matrices are CV_32F.
        virtual bool leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients);

        /// \brief get the calibration matrices of the right camera in opencv format.  Matrices are CV_32F.
        virtual bool rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients);

        /// \brief get the extrinsic matrices, i.e., transformation from left to right camera. Matrices are CV_32F.
        virtual bool extrinsic(cv::Mat &_rotation, cv::Mat &_translation);

        /// \brief get the extrinsic matrices, i.e., transformation from left to right camera.
        virtual bool extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation);

        /// \brief get disparty-to-depth parameter typical from RGB-D devices.
        virtual bool disparityToDepthParam(double &_dispToDepth);

        /// \brief default destructor.
        virtual ~StereoCamera() {};

        virtual bool colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point) = 0;

	};	//	class StereoCamera
}	//	namespace mico 

#include "StereoCamera.inl"

#endif	//	RGBDSLAM_VISION_STEREOCAMERA_H_
