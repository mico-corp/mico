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


#include <mico/cameras/StereoCameras/StereoCameraRos.h>
#include <pcl/features/integral_image_normal.h>

#ifdef RGBDTOOLS_USE_ROS
    #include <cv_bridge/cv_bridge.h>
#endif

namespace mico {

    //-----------------------------------------------------------------------------------------------------------------
    StereoCameraRos::~StereoCameraRos() {
		        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::init(const cjson::Json & _json){
        #ifdef RGBDTOOLS_USE_ROS
            if(!ros::isInitialized()){
                std::cout << "[STEREOCAMERA][ROS] Stereo Camera ROS cannot initialize ros library, please init it anywhere before calling StereoCameraRos::init() method" <<std::endl;
                return false;
            }
            

            ros::NodeHandle nh;
            image_transport::ImageTransport it(nh);
            if(_json.contains("left") && ((std::string)_json["left"] != "")){
                mSubscriberLeft = it.subscribe(_json["left"], 1, &StereoCameraRos::leftCallback, this);
            }
            if(_json.contains("right") && ((std::string) _json["right"] != "")){
                mSubscriberRight = it.subscribe(_json["right"], 1, &StereoCameraRos::rightCallback, this);
            }
            if(_json.contains("depth") && ((std::string) _json["depth"] != "")){
                mSubscriberDepth = it.subscribe(_json["depth"], 1, &StereoCameraRos::depthCallback, this);
            }
            //if(_json.contains("cloud")){
            //    mSubscriberCloud = it.subscribe(_json["cloud"], &StereoCameraRos::cloudCallback, this);
            //}
        return true;

        #else
            return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::rgb(cv::Mat & _left, cv::Mat & _right){
        #ifdef RGBDTOOLS_USE_ROS
            _left = mLastRGB;
            _right = mRight;
            return true;
        #else
            return false;
        #endif
    }

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::depth(cv::Mat & _depth){
        #ifdef RGBDTOOLS_USE_ROS
            _depth = mLastDepthInColor;
            return true;
        #else
            return false;
        #endif
    }

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::grab(){
        #ifdef RGBDTOOLS_USE_ROS
            return true;
        #else
            return false;
        #endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::cloud(pcl::PointCloud<pcl::PointXYZ>& _cloud) {
        #ifdef RGBDTOOLS_USE_ROS
            return true;
        #else
            return false;
        #endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::cloud(pcl::PointCloud<pcl::PointXYZRGB>& _cloud) {
       #ifdef RGBDTOOLS_USE_ROS
            return true;
        #else
            return false;
        #endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal>& _cloud) {
       #ifdef RGBDTOOLS_USE_ROS
            return true;
        #else
            return false;
        #endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::cloud(pcl::PointCloud<pcl::PointNormal>& _cloud) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        mMatrixLeft.copyTo(_intrinsic);
        mDistCoefLeft.copyTo(_coefficients);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        mMatrixRight.copyTo(_intrinsic);
        mDistCoefRight.copyTo(_coefficients);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::extrinsic(cv::Mat &_rotation, cv::Mat &_translation) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::disparityToDepthParam(double &_dispToDepth){
        _dispToDepth = mDispToDepth;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRos::colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point) {
        return false;
    }

    #ifdef RGBDTOOLS_USE_ROS
        //---------------------------------------------------------------------------------------------------------------------
        void StereoCameraRos::leftCallback(const sensor_msgs::Image::ConstPtr &_msg){
            mLastRGB = cv_bridge::toCvCopy(_msg, "bgr8")->image;
        }

        //---------------------------------------------------------------------------------------------------------------------
        void StereoCameraRos::rightCallback(const sensor_msgs::Image::ConstPtr &_msg){
            mRight = cv_bridge::toCvCopy(_msg, "bgr8")->image;
        }

        //---------------------------------------------------------------------------------------------------------------------
        void StereoCameraRos::depthCallback(const sensor_msgs::Image::ConstPtr &_msg){
            mLastDepthInColor = cv_bridge::toCvCopy(_msg, _msg->encoding)->image;
        }
    #endif

}	//	namespace mico 