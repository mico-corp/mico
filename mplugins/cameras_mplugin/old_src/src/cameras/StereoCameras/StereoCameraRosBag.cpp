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


#include <mico/cameras/StereoCameras/StereoCameraRosBag.h>
#include <pcl/features/integral_image_normal.h>
#ifdef RGBDTOOLS_USE_ROS
    #include <cv_bridge/cv_bridge.h>
#endif

namespace mico {

    //-----------------------------------------------------------------------------------------------------------------
    StereoCameraRosBag::~StereoCameraRosBag() {
		        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::init(const cjson::Json & _json){
        #ifdef RGBDTOOLS_USE_ROS
            mBag.open(_json["bag_file"], rosbag::bagmode::Read);

            if(_json.contains("left") && ((std::string)_json["left"] != "")){
                leftView = new rosbag::View(mBag, rosbag::TopicQuery(_json["left"]));
                leftIt = leftView->begin();
                mHasLeft = true;
            }
            if(_json.contains("right") && ((std::string) _json["right"] != "")){
                rightView = new rosbag::View(mBag, rosbag::TopicQuery(_json["right"]));
                rightIt = rightView->begin();
                mHasRight = true;
            }
            if(_json.contains("depth") && ((std::string) _json["depth"] != "")){
                depthView = new rosbag::View(mBag, rosbag::TopicQuery(_json["depth"]));
                depthIt = depthView->begin();
                mHasDepth = true;
            }
            //if(_json.contains("cloud")){
            //    mSubscriberCloud = it.subscribe(_json["cloud"], &StereoCameraRosBag::cloudCallback, this);
            //}

            // Load Calibration files if path exist
            if(_json.contains("calibFile") && std::string(_json["calibFile"]) != ""){
                mHasCalibration = true;

                cv::FileStorage fs((std::string)_json["calibFile"], cv::FileStorage::READ);

                fs["MatrixLeft"]            >> mMatrixLeft;
                fs["DistCoeffsLeft"]        >> mDistCoefLeft;
                fs["MatrixRight"]           >> mMatrixRight;
                fs["DistCoeffsRight"]       >> mDistCoefRight;
                fs["Rotation"]              >> mRot;
                fs["Translation"]           >> mTrans;
                fs["DisparityToDepthScale"] >> mDispToDepth;

            }else{
                mHasCalibration = false;
            }

            return true;
        #else
            return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::rgb(cv::Mat & _left, cv::Mat & _right){
        #ifdef RGBDTOOLS_USE_ROS
            _left = mLastRGB;
            _right = mRight;
            return true;
        #else
            return false;
        #endif
    }

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::depth(cv::Mat & _depth){
        #ifdef RGBDTOOLS_USE_ROS
            _depth = mLastDepthInColor;
            return true;
        #else
            return false;
        #endif
    }

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::grab(){
        #ifdef RGBDTOOLS_USE_ROS

            if(leftIt != leftView->end()){
                auto msg = leftIt->instantiate<sensor_msgs::Image>();
                mLastRGB = cv_bridge::toCvCopy(msg, "bgr8")->image;
                leftIt++;
            }else{
                if(mHasLeft)
                    return false;
            }

            if(rightIt != rightView->end()){
                auto msg = leftIt->instantiate<sensor_msgs::Image>();
                mRight = cv_bridge::toCvCopy(msg, "bgr8")->image;
                rightIt++;
            }else{
                if(mHasRight)
                    return false;
            }

            if(depthIt != depthView->end()){
                auto msg = depthIt->instantiate<sensor_msgs::Image>();
                mLastDepthInColor = cv_bridge::toCvCopy(msg, "mono16")->image;
                depthIt++;
            }else{
                if(mHasDepth)
                    return false;
            }

            return true;
        #else
            return false;
        #endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::cloud(pcl::PointCloud<pcl::PointXYZ>& _cloud) {
        #ifdef RGBDTOOLS_USE_ROS
            return true;
        #else
            return false;
        #endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::cloud(pcl::PointCloud<pcl::PointXYZRGB>& _cloud) {
       #ifdef RGBDTOOLS_USE_ROS
            for (int dy = 0; dy < mLastDepthInColor.rows; dy++) {
                for (int dx = 0; dx < mLastDepthInColor.cols; dx++) {
					// Retrieve the 16-bit depth value and map it into a depth in meters
                    uint16_t depth_value = mLastDepthInColor.at<uint16_t>(dy, dx);
					float depth_in_meters = depth_value * mDispToDepth;
                    // Set invalid pixels with a depth value of zero, which is used to indicate no data
                    pcl::PointXYZRGB point;
                    if (depth_value == 0) {
                        point.x = NAN;
                        point.y = NAN;
                        point.z = NAN;
                    }
                    else {
                        // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                        cv::Point2f depth_pixel(dx, dy);
                        float x = (depth_pixel.x - mMatrixLeft.at<float>(0,2)) / mMatrixLeft.at<float>(0,0);
                        float y = (depth_pixel.y - mMatrixLeft.at<float>(1,2)) / mMatrixLeft.at<float>(1,1);

                        cv::Point3f depth_point(x*depth_in_meters, y*depth_in_meters, depth_in_meters);
                        point.x = depth_point.x;
                        point.y = depth_point.y;
                        point.z = depth_point.z;
                        auto rgb = mLastRGB.at<cv::Vec3b>(dy, dx);
                        point.r = rgb[2];
                        point.g = rgb[1];
                        point.b = rgb[0];

                    }

					_cloud.push_back(point);
                }
            }
            setOrganizedAndDense(_cloud);
            return true;

        #else
            return false;
        #endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal>& _cloud) {
       #ifdef RGBDTOOLS_USE_ROS
            pcl::PointCloud<pcl::PointXYZRGB> cloudWoNormals;
			if (!cloud(cloudWoNormals)) {
                std::cout << "[STEREOCAMERA][ROSBAG] Cannot compute cloud" << std::endl;
				return false;
			}

            if(cloudWoNormals.size() == 0){
                std::cout << "[STEREOCAMERA][REALSENSE] Empty cloud, can't compute normals" << std::endl;
                _cloud.resize(0);
                return false;
            }

            pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
			ne.setInputCloud(cloudWoNormals.makeShared());
			ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
			ne.setMaxDepthChangeFactor(0.02f);
			ne.setNormalSmoothingSize(10.0f);
			ne.compute(_cloud);

			// Fill XYZ and RGB of cloud
			for (unsigned i = 0; i < _cloud.size(); i++) {
				_cloud[i].x = cloudWoNormals[i].x;
				_cloud[i].y = cloudWoNormals[i].y;
				_cloud[i].z = cloudWoNormals[i].z;
				_cloud[i].r = cloudWoNormals[i].r;
				_cloud[i].g = cloudWoNormals[i].g;
				_cloud[i].b = cloudWoNormals[i].b;
			}
            setOrganizedAndDense(_cloud);

            return true;
        #else
            return false;
        #endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::cloud(pcl::PointCloud<pcl::PointNormal>& _cloud) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        mMatrixLeft.copyTo(_intrinsic);
        mDistCoefLeft.copyTo(_coefficients);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        mMatrixRight.copyTo(_intrinsic);
        mDistCoefRight.copyTo(_coefficients);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::extrinsic(cv::Mat &_rotation, cv::Mat &_translation) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::disparityToDepthParam(double &_dispToDepth){
        _dispToDepth = mDispToDepth;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point) {
        // Retrieve the 16-bit depth value and map it into a depth in meters
        uint16_t depth_value = mLastDepthInColor.at<uint16_t>(_pixel.y, _pixel.x);
        float depth_in_meters = depth_value * mDispToDepth;
        // Set invalid pixels with a depth value of zero, which is used to indicate no data
        pcl::PointXYZRGB point;
        if (depth_value == 0) {
            return false;
        }
        else {
            // Map from pixel coordinates in the depth image to pixel coordinates in the color image
            float x = (_pixel.x - mMatrixLeft.at<float>(0,2)) / mMatrixLeft.at<float>(0,0);
            float y = (_pixel.y - mMatrixLeft.at<float>(1,2)) / mMatrixLeft.at<float>(1,1);

            cv::Point3f depth_point(x*depth_in_meters, y*depth_in_meters, depth_in_meters);

            _point.x = depth_point.x;
            _point.y = depth_point.y;
            _point.z = depth_point.z;
            return true;
        }
    }
}	//	namespace mico 