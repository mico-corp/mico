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


#ifndef MICO_BASE_VISION_STEREOCAMERAS_STEREOCAMERAZED_H_
#define MICO_BASE_VISION_STEREOCAMERAS_STEREOCAMERAZED_H_


#include <mico/cameras/StereoCamera.h>

#ifdef HAS_ZED_SDK
    #include <zed/Camera.hpp>
#endif

namespace mico {
	/// \brief Wrapper of ZED stereo camera.
	class StereoCameraZed :public StereoCamera {
	public:
		/// \brief Initialize the ZED stereo camera using a config file.
		/// Config file must have following structure.
		///
		/// \code
		///		{
		///			"mode":"default"	// Now is the only option.
		///		}
		/// \endcode
		///
		/// \param _json: Configuration file.
		bool init(const cjson::Json &_json = "");

		/// \brief Get pair of rgb images from the camera.
		/// \param _left: referente to a container for the left image.
		/// \param _right: referente to a container for the right image.
		bool rgb(cv::Mat &_left, cv::Mat &_right);

		/// \brief Get depth image from the camera.
		/// \param _depth: referente to a container for the depth image
		bool depth(cv::Mat &_depth);

		/// \brief Retrieve an uncolorized point cloud from the camera
		bool cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud);

		/// \brief Retrieve a colorized point cloud from the camera
		bool cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud);

		/// \brief [DUMMY] Override of cloud method of StereoCamera.
		virtual bool cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud) {
			std::cout << "[STEREO CAMERA][ZED] Cloud method is not currently implemented in ZED cameras" << std::endl;
			return false;
		}

		/// \brief [DUMMY] Override of cloud method of StereoCamera.
		virtual bool cloud(pcl::PointCloud<pcl::PointNormal> &_cloud) {
			std::cout << "[STEREO CAMERA][ZED] Cloud method is not currently implemented in ZED cameras" << std::endl;
			return false;
		}

		/// \brief The function grabs a new image, rectifies it and computes the disparity map and optionally the depth map.
		bool grab();

        bool colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point){return false;}
	private:
#ifdef HAS_ZED_SDK
		sl::zed::Camera *mZedCamera = nullptr;
#endif	// HAS_ZED_SDK

		bool mHasRGB = false, mComputedDepth = false, mComputedCloudXYZ = false, mComputedCloudXYZRGB = false, mComputedCloudXYZRGBNormal = false, mComputedCloudNormal = false;
		cv::Mat mLeftFrame, mRightFrame, mDepth;
		pcl::PointCloud<pcl::PointXYZ> mCloudXYZ;
		pcl::PointCloud<pcl::PointXYZRGB> mCloudXYZRGB;
		pcl::PointCloud<pcl::PointXYZRGBNormal> mCloudXYZRGBNormal;
		pcl::PointCloud<pcl::PointNormal> mCloudXYZNormal;

	};
}	//	namespace mico 

#endif		// RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAZED_H_
