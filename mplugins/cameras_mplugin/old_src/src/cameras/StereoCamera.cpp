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

#include <mico/cameras/StereoCamera.h>

#include <mico/cameras/StereoCameras/StereoCameraVirtual.h>
#include <mico/cameras/StereoCameras/StereoCameraZED.h>
#include <mico/cameras/StereoCameras/StereoCameraCustom.h>
#include <mico/cameras/StereoCameras/StereoCameraRealSense.h>
#include <mico/cameras/StereoCameras/StereoCameraKinect.h>
#include <mico/cameras/StereoCameras/StereoCameraRos.h>
#include <mico/cameras/StereoCameras/StereoCameraRosBag.h>


namespace mico {
	//---------------------------------------------------------------------------------------------------------------------
	StereoCamera * StereoCamera::create(eModel _type) {
		if (_type == eModel::Virtual) {
			return new StereoCameraVirtual();
		}else if (_type == eModel::Zed) {
			return new StereoCameraZed();
		}else if (_type == eModel::ArtecEva) {
            std::cerr << "[STERECAMERA] Deprecated model: Artec EVA" << std::endl;
            return nullptr;
		}else if (_type == eModel::Custom) {
			return new StereoCameraCustom();
		}else if (_type == eModel::RealSense) {
            return new StereoCameraRealSense();
        }else if (_type == eModel::Kinect) {
            return new StereoCameraKinect();
        }else if (_type == eModel::ROS) {
            return new StereoCameraRos();
        }else if (_type == eModel::rosbag) {
            return new StereoCameraRosBag();
        }else {
            std::cerr << "[STEREOCAMERA]  unknown model type" << std::endl;
			return nullptr;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
	StereoCamera * StereoCamera::create(std::string _type) {
		if (_type == "virtual") {
			return new StereoCameraVirtual();
		}else if (_type == "zed") {
			return new StereoCameraZed();
		}else if (_type == "artec") {
            std::cerr << "[STERECAMERA] Deprecated model: Artec EVA" << std::endl;
            return nullptr;
		}else if (_type == "custom") {
			return new StereoCameraCustom();
		}else if (_type == "realsense") {
            return new StereoCameraRealSense();
        }else if (_type == "kinect") {
            return new StereoCameraKinect();
        }else if (_type == "ROS" || _type == "ros") {
            return new StereoCameraRos();
        }else if (_type == "rosbag") {
            return new StereoCameraRosBag();
        }else {
            std::cerr << "[STEREOCAMERA]  unknown model type" << std::endl;
			return nullptr;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCamera::leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        std::cerr << "[STEREOCAMERA] leftCalibration method not implemented for given point type." << std::endl;
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCamera::rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        std::cerr << "[STEREOCAMERA] rightCalibration method not implemented for given point type." << std::endl;
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCamera::extrinsic(cv::Mat &_rotation, cv::Mat &_translation) {
        std::cerr << "[STEREOCAMERA] extrinsic method not implemented for given point type." << std::endl;
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCamera::extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation) {
        std::cerr << "[STEREOCAMERA] extrinsic method not implemented for given point type." << std::endl;
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCamera::disparityToDepthParam(double &_dispToDepth) {
        std::cerr << "[STEREOCAMERA] cloud method not implemented for given point type." << std::endl;
        return false;
    }

	//---------------------------------------------------------------------------------------------------------------------
}	//	namespace mico 
