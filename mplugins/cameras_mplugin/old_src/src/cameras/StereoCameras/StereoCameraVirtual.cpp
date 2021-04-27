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


#include <mico/cameras/StereoCameras/StereoCameraVirtual.h>

#include <fstream>
#include <cstdio>

#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

using namespace cv;
using namespace pcl;
using namespace std;

namespace mico {
	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::init(const cjson::Json &_json) {
		if (_json.isObject()) {
            if(!_json.contains("input")){
                std::cout <<"[STEREOCAMERA][VIRTUAL] Need to provide the input field in config file" << std::endl;
                return false;
            }

            bool hasSmth = false;
            if(_json["input"].contains("left")){
                mLeftImageFilePathTemplate = std::string(_json ["input"]["left"]);
                hasSmth = true;
            }
            if(_json["input"].contains("right")){
                mRightImageFilePathTemplate = std::string(_json["input"]["right"]);
                hasSmth = true;
            }
            if(_json["input"].contains("depth")){
                mDepthImageFilePathTemplate = std::string(_json["input"]["depth"]);
                hasSmth = true;
            }
            if(_json["input"].contains("pointCloud")){
                mPointCloudFilePathTemplate = std::string(_json["input"]["pointCloud"]);
                hasSmth = true;
            }

            if(!hasSmth)
                return false;

            if(_json.contains("firstIdx"))
                mFrameCounter   = (int) _json["firstIdx"];   

            if(_json.contains("stepIdx"))
                mStepIdx        = (int) _json["stepIdx"];   
            
            if(_json.contains("loop_dataset"))
                mLoopDataset = (bool) _json["loop_dataset"];
            
            

            // Load Calibration files if path exist
            if(_json.contains("calibFile") && std::string(_json["calibFile"]) != ""){
                mHasCalibration = true;

                FileStorage fs((std::string)_json["calibFile"], FileStorage::READ);

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
		}
		else {
			std::cout << "[STEREO CAMERA][VIRTUAL] Virtual stereo camera couldn't be initialized" << std::endl;
			return false;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::rgb(Mat & _left, Mat & _right) {
		if (mLeftImageFilePathTemplate == "" && mRightImageFilePathTemplate == "") {
			return false;
		}

        _left = mLeft;
        _right = mRight;

        if (_right.rows == 0 && _left.rows == 0) {
			return false;
		}
		else {
            if (_right.rows == 0 || _left.rows == 0) {
				//std::cout << "[STEREO CAMERA][VIRTUAL] Warning, this camera only provide one color image\n";
			}

			return true;
		}

	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::depth(Mat & _depth) {
		if (mDepthImageFilePathTemplate == "") {
			return false;
		}

        _depth = mDepth;

		if (_depth.rows == 0)
			return false;
		else
			return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::cloud(PointCloud<PointXYZ>& _cloud) {

		if (mPointCloudFilePathTemplate == "") {
			if (mDepthImageFilePathTemplate == "") {
				return false;
			}
            else {
                depthToPointcloud(mDepth, _cloud);
				return true;
			}
		}
		else {
			char bufferString[1024];
              int bytes = std::sprintf(bufferString, mPointCloudFilePathTemplate.c_str(), mFrameCounter);

            string imagePath = (std::string(bufferString));

			pcl::io::loadPCDFile(imagePath, _cloud);
			if (_cloud.size() != 0) {
				return true;
			}
		}
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud) {
		if (mPointCloudFilePathTemplate != "") {
			char bufferString[1024];
              int bytes = std::sprintf(bufferString, mPointCloudFilePathTemplate.c_str(), mFrameCounter);
            
            string imagePath = (std::string(bufferString));

			pcl::io::loadPCDFile(imagePath, _cloud);
			if (_cloud.size() != 0) {
				return true;
			}
        }else if(mDepthImageFilePathTemplate != ""){
            depthToPointcloud(mDepth, _cloud);
            return true;
        }
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::cloud(pcl::PointCloud<pcl::PointNormal> &_cloud) {
		if (mPointCloudFilePathTemplate != "") {
			char bufferString[1024];
              int bytes = std::sprintf(bufferString, mPointCloudFilePathTemplate.c_str(), mFrameCounter);
            string imagePath = (std::string(bufferString));

			pcl::io::loadPCDFile(imagePath, _cloud);
			if (_cloud.size() != 0) {
				return true;
			}
		}
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud) {
		if (mPointCloudFilePathTemplate != "") {
			char bufferString[1024];
              int bytes = std::sprintf(bufferString, mPointCloudFilePathTemplate.c_str(), mFrameCounter);
            string imagePath = (std::string(bufferString));

            pcl::io::loadPCDFile(imagePath, _cloud);
            if (_cloud.size() == 0) {
                return false;
            }else{
                pcl::PCLPointCloud2* header = new pcl::PCLPointCloud2;
                pcl::PCDReader reader;
                reader.readHeader(imagePath, *header);

                bool hasNormals = false;
                for(unsigned i = 0; i < header->fields.size(); i++){
                    if(header->fields[i].name.find("normal")!= std::string::npos){
                        hasNormals = true;
                    }
                }

                if(hasNormals){
                    return true;
                }else{
                    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
                    ne.setInputCloud(_cloud.makeShared());
                    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
                    ne.setMaxDepthChangeFactor(0.02f);
                    ne.setNormalSmoothingSize(10.0f);
                    ne.compute(_cloud);
                    return true;
                }
            }
        }else if (mDepthImageFilePathTemplate != "") {
            pcl::PointCloud<pcl::PointXYZRGB> colorCloud;
            depthToPointcloud(mDepth, colorCloud);
            pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
            ne.setInputCloud(colorCloud.makeShared());
            ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
            ne.setMaxDepthChangeFactor(0.02f);
            ne.setNormalSmoothingSize(10.0f);
            ne.compute(_cloud);
            for(unsigned i = 0; i < _cloud.size(); i++){
                _cloud[i].x = colorCloud[i].x;
                _cloud[i].y = colorCloud[i].y;
                _cloud[i].z = colorCloud[i].z;
                _cloud[i].r = colorCloud[i].r;
                _cloud[i].g = colorCloud[i].g;
                _cloud[i].b = colorCloud[i].b;
            }
            return true;   // 666 not implemented but should be possible
        }
        else {

            return false;
        }
    }
    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraVirtual::leftCalibration(Mat &_intrinsic, Mat &_coefficients){
        _intrinsic = mMatrixLeft;
        _coefficients = mDistCoefLeft;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraVirtual::rightCalibration(Mat &_intrinsic, Mat &_coefficients){
        _intrinsic = mMatrixRight;
        _coefficients = mDistCoefRight;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraVirtual::extrinsic(Mat &_rotation, Mat &_translation){
        _rotation = mRot;
        _translation = mTrans;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraVirtual::extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation){
        _rotation = Eigen::Matrix3f((float*)mRot.data);
        _translation = Eigen::Vector3f((float*)mTrans.data);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraVirtual::disparityToDepthParam(double &_dispToDepth){
        _dispToDepth = mDispToDepth;
        return true;
    }

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::grab() {
		mFrameCounter += mStepIdx;

        if (mLeftImageFilePathTemplate != "") {
            char bufferString[1024];
              int bytes = std::sprintf(bufferString, mLeftImageFilePathTemplate.c_str(), mFrameCounter);
            string imagePath = (std::string(bufferString));

            mLeft = imread(imagePath);
            if(mLeft.rows == 0){
                if(mLoopDataset){
                    std::cout << "[STEREO CAMERA][VIRTUAL] Cant load image, assumed reached end of dataset, restarting counter\n";
                    mFrameCounter = 0;
                }
            }
        }

        if (mRightImageFilePathTemplate != "") {
            char bufferString[1024];
              int bytes = std::sprintf(bufferString, mRightImageFilePathTemplate.c_str(), mFrameCounter);
            string imagePath = (std::string(bufferString));
            mRight = imread(imagePath.substr(0, imagePath.size() - 1));
        }

        if(mDepthImageFilePathTemplate != ""){
            char bufferString[1024];
              int bytes = std::sprintf(bufferString, mDepthImageFilePathTemplate.c_str(), mFrameCounter);
            string imagePath = (std::string(bufferString));
 
            mDepth = imread(imagePath, -1);
        }


		return true;
	}

    //---------------------------------------------------------------------------------------------------------------------
    void StereoCameraVirtual::depthToPointcloud(Mat & _depth, PointCloud<PointXYZ>& _cloud) {
        // Fake parameters
        int cx = mMatrixLeft.at<float>(0,2);
        int cy = mMatrixLeft.at<float>(1,2);;
        double fx = mMatrixLeft.at<float>(0,0);
        double fy = mMatrixLeft.at<float>(1,1);
        std::cout << "cx= " << cx << "cy= " << cy << "fx= " << fx << "fy= " << fy << std::endl;
        for (int i = 0; i < _depth.rows; i++) {
            for (int j = 0; j < _depth.cols; j++) {
                double z = double(_depth.at<unsigned short>(i*_depth.cols + j)) * mDispToDepth;
                if (!z)
                    continue;
                double x = double(j - cx)*z / fx;
                double y = double(i - cy)*z / fy;
                _cloud.push_back(PointXYZ(x, y, z));
            }
        }
        _cloud.is_dense = false; // 666 TODO: cant set to true if wrong points are set to NaN.
        _cloud.width = mLeft.cols;
        _cloud.height = mLeft.rows;

    }

    //---------------------------------------------------------------------------------------------------------------------
    void StereoCameraVirtual::depthToPointcloud(Mat & _depth, PointCloud<PointXYZRGB>& _cloud) {
        // Fake parameters
        int cx = mMatrixLeft.at<float>(0,2);
        int cy = mMatrixLeft.at<float>(1,2);;
        double fx = mMatrixLeft.at<float>(0,0);
        double fy = mMatrixLeft.at<float>(1,1);
        for (int i = 0; i < _depth.rows; i++) {
            for (int j = 0; j < _depth.cols; j++) {
                double z = double(_depth.at<unsigned short>(i*_depth.cols + j)) * mDispToDepth;
                PointXYZRGB p;
                if (z){
                    double x = double(j - cx)*z / fx;
                    double y = double(i - cy)*z / fy;

                    p.x = x;
                    p.y = y;
                    p.z = z;
                    p.r = mLeft.at<Vec3b>(i,j)[2];
                    p.g = mLeft.at<Vec3b>(i,j)[1];
                    p.b = mLeft.at<Vec3b>(i,j)[0];
                }else{
                    p.x = NAN;
                    p.y = NAN;
                    p.z = NAN;
                    p.r = 0;
                    p.g = 0;
                    p.b = 0;
                }

                _cloud.push_back(p);
            }
        }

        _cloud.is_dense = false; // 666 TODO: cant set to true if wrong points are set to NaN.
        _cloud.width = mLeft.cols;
        _cloud.height = mLeft.rows;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraVirtual::colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point){
        // Retrieve the 16-bit depth value and map it into a depth in meters
        uint16_t depth_value = mDepth.at<uint16_t>(_pixel.y, _pixel.x);
        float depth_in_meters = depth_value * mDispToDepth;
        // Set invalid pixels with a depth value of zero, which is used to indicate no data
        if (depth_value == 0) {
            return false;
        }
        else {
            // 666 Assuming that it is undistorted which is for intel real sense F200 and depth is in color CS...
            _point.x = (_pixel.x - mMatrixLeft.at<float>(0,2))/mMatrixLeft.at<float>(0,0)*depth_in_meters;
            _point.y = (_pixel.y - mMatrixLeft.at<float>(1,2))/mMatrixLeft.at<float>(1,1)*depth_in_meters;
            _point.z = depth_in_meters;
            return true;
        }
    }
}	//	namespace mico 
