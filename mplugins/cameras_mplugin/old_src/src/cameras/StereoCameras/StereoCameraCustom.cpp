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


#include <mico/cameras/StereoCameras/StereoCameraCustom.h>
#ifdef USE_LIBELAS
#include <libelas/elas.h>
#endif
#include <mico/cameras/StereoCameras/ParallelFeatureMatcher.h>
#include <thread>
#include <chrono>

using namespace std;
using namespace cv;
using namespace pcl;

namespace mico {
	StereoCameraCustom::StereoCameraCustom() {
		mCloudXYZ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		mCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		mCloudXYZRGBNormal = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		mCloudXYZNormal = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
	}
	//---------------------------------------------------------------------------------------------------------------------
	StereoCameraCustom::~StereoCameraCustom() {
		if (mCameraLeft)
			delete mCameraLeft;

		if (mCameraRight)
			delete mCameraRight;

#ifdef HAS_ZED_SDK
		if (mZedCamera)
			delete mZedCamera;
#endif
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::init(const cjson::Json &_json) {
		if (!_json.isObject()) {
			std::cout << "[STEREO CAMERA][CUSTOM] This kind of camera needs a configuration file" << std::endl;
			return false;
		}

		if (!configureDevice(_json["device"])) {
			return false;
		}

        if(_json.contains("cloud")){
            if (!decodeCloudType(_json["cloud"])) {
                return false;
            }
        }

		std::cout << "[STEREO CAMERA][CUSTOM] Configured and ready to be used" << std::endl;
		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::rgb(cv::Mat &_left, cv::Mat &_right) {
		if (mHasRGB) {
			_left = mLeftFrame;
			_right = mRightFrame;
			return true;
		}
		else {
			switch (mType) {
			case eDeviceType::opencv: {
				//	 Retrieve data from OpenCV drivers
				bool res = mCameraLeft->retrieve(mLeftFrame);
				if(mCameraRight){
					res &= mCameraRight->retrieve(mRightFrame);
				}
				
				// Copy to out variables.
				_left = mLeftFrame;
				_right = mRightFrame;

				// Set flag to reuse data until next grab.
				mHasRGB = true;
				return res;
			}
			case eDeviceType::zed: {
#ifdef HAS_ZED_SDK
				// Retrieve data from ZED SDK
				sl::zed::Mat left = mZedCamera->retrieveImage(sl::zed::SIDE::LEFT);
				sl::zed::Mat right = mZedCamera->retrieveImage(sl::zed::SIDE::RIGHT);

				if (left.width == 0 || right.width == 0)
					return false;

				mLeftFrame = cv::Mat(left.height, left.width, CV_8UC4);
				memcpy(mLeftFrame.data, left.data, left.width*left.height * 4 * sizeof(uchar));

				mRightFrame = cv::Mat(right.height, right.width, CV_8UC4);
				memcpy(mRightFrame.data, right.data, right.width*right.height * 4 * sizeof(uchar));

				// Copty to out variables
				_left = mLeftFrame;
				_right = mRightFrame;

				// Set flag to reuse data until next grab.
				mHasRGB = true;
				return true;
#else
				cv::Mat doubleImage;
				bool res = mCameraLeft->retrieve(doubleImage);

				mLeftFrame = doubleImage(Rect(0, 0, doubleImage.cols / 2, doubleImage.rows)).clone();
				mRightFrame = doubleImage(Rect(doubleImage.cols / 2, 0, doubleImage.cols / 2, doubleImage.rows)).clone();
				// Copy to out variables.
				_left = mLeftFrame;
				_right = mRightFrame;

				// Set flag to reuse data until next grab.
				mHasRGB = true;
				return res;
#endif	// HAS_ZED_SDK
			}
			default:
				return false;
			}
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::depth(cv::Mat &_depth) {
		if (mComputedDepth) {	// Reuse data if it is previously computed.
			_depth = mDepth;
			return true;
		}
		else {
			// Switch between methods
			switch (mDepthAlgorithm) {
			case eDepthAlgorithm::elas:
#ifdef USE_LIBELAS
				if (!disparityLibelas(mDepth)) {
					return false;
				}
#else
				return false;
#endif
				break;
			case eDepthAlgorithm::BM:
				if (!disparityCvBm(mDepth)) {
					return false;
				}
				break;
			case eDepthAlgorithm::SGBM:
				if (!disparityCvSgbm(mDepth)) {
					return false;
				}
				break;
			default:
				std::cout << "[STEREO CAMERAS][CUSTOM] Not configured any depth algorithm, check configuration file" << std::endl;
				return false;
				break;
			}

			// Copy to out variables
			_depth = mDepth;
			mComputedDepth = true;
			return true;
		}

	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::grab() {
		// Reset variables in order not to reuse old data
		mHasRGB = false, mComputedDepth = false, mComputedCloudXYZ = false, mComputedCloudXYZRGB = false, mComputedCloudXYZRGBNormal = false, mComputedCloudNormal = false;

		// Switch between camera implementations.
		switch (mType) {
		case eDeviceType::opencv: {
			bool res = mCameraLeft->grab();
			if (mCameraRight) {
				res &= mCameraRight->grab();
			}
			return res;
		}
		case eDeviceType::zed:
#ifdef HAS_ZED_SDK
			return mZedCamera->grab(sl::zed::SENSING_MODE::FULL, 0, 0);
#else
			return mCameraLeft->grab();
#endif	// HAS_ZED_SDK
		default:
			return false;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud) {	//666 Clean this
		if (mComputedCloudXYZ) {
			_cloud = *mCloudXYZ;
			return true;
		}
		else {
			mCloudXYZ->clear();
			switch (mCloudType) {
			case eCloudType::Sparse:
				if (!computeCloudSparse(mCloudXYZ)) {
					return false;
				}
				break;
			case eCloudType::Dense:
				if (!computeCloudDense(mCloudXYZ)) {
					return false;
				}
				break;
			default:
				std::cout << "[STEREO CAMERAS][CUSTOM] Cloud type not defined" << std::endl;
				return false;
				break;
			}

			// Copy to out variable.
			_cloud = *mCloudXYZ;
			mComputedCloudXYZ = true;
			return true;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud) {	//666 Clean this
		if (mComputedCloudXYZRGB) {
			_cloud = *mCloudXYZRGB;
			return true;
		}
		else {
			mCloudXYZRGB->clear();
			switch (mCloudType) {
			case eCloudType::Sparse:
				if (!computeCloudSparse(mCloudXYZRGB)) {
					return false;
				}
				break;
			case eCloudType::Dense:
				if (!computeCloudDense(mCloudXYZRGB)) {
					return false;
				}
				break;
			default:
				std::cout << "[STEREO CAMERAS][CUSTOM] Cloud type not defined" << std::endl;
				return false;
				break;
			}

			// Copy to out variable.
			_cloud = *mCloudXYZRGB;
			mComputedCloudXYZRGB = true;
			return true;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud) {	//666 Clean this
		if (mComputedCloudXYZRGBNormal) {
			_cloud = *mCloudXYZRGBNormal;
			return true;
		}
		else {
			mCloudXYZRGBNormal->clear();
			switch (mCloudType) {
			case eCloudType::Sparse:
				if (!computeCloudSparse(mCloudXYZRGBNormal)) {
					return false;
				}
				break;
			case eCloudType::Dense:
				if (!computeCloudDense(mCloudXYZRGBNormal)) {
					return false;
				}
				break;
			default:
				std::cout << "[STEREO CAMERAS][CUSTOM] Cloud type not defined" << std::endl;
				return false;
				break;
			}

			// Copy to out variable.
			_cloud = *mCloudXYZRGBNormal;
			mComputedCloudXYZRGBNormal = true;
			return true;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::cloud(pcl::PointCloud<pcl::PointNormal> &_cloud) {
		std::cout << "[STEREO CAMERAS][CUSTOM] Cloud with normals is not currently implemented" << std::endl;
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	// Private methods
	bool StereoCameraCustom::configureDevice(const cjson::Json &_json) {
		if (_json["type"] == "opencv") {
			// Getting source for left image
			if (_json["left"].isString()) {
				mCameraLeft = new cv::VideoCapture(std::string(_json["left"]));
			}
			else if (_json["left"].isNumber()) {
				mCameraLeft = new cv::VideoCapture((int)_json["left"]);
			}
			else {
				std::cout << "[STEREO CAMERA][CUSTOM] Can't recognize source for left images." << std::endl;
				return false;
			}
			if (!mCameraLeft) {
				std::cout << "[STEREO CAMERA][CUSTOM] Error opening left camera." << std::endl;
				return false;
			}

			// Getting source for right image
			if (_json["right"].isString()) {
				mCameraRight = new cv::VideoCapture(std::string(_json["right"]));
			}
			else if (_json["right"].isNumber()) {
				mCameraRight = new cv::VideoCapture((int)_json["right"]);
			}
			else {
				std::cout << "[STEREO CAMERA][CUSTOM] Right images not enabled." << std::endl;
			}

			mType = eDeviceType::opencv;

			if (_json.contains("calibFile")) {
				mIsCalibrated = loadCalibrationFile(_json["calibFile"]);
			}
			else {
				mIsCalibrated = false;
			}
		}
		else if (_json["type"] == "zed") {
#ifdef HAS_ZED_SDK
			// Opening zed camera, but only for getting pair of images.
			mZedCamera = new sl::zed::Camera(sl::zed::ZEDResolution_mode::VGA);
			auto errCode = mZedCamera->init(sl::zed::MODE::PERFORMANCE);
			mType = eDeviceType::zed;
			if (errCode == sl::zed::ERRCODE::SUCCESS) {
				if (_json.contains("calibFile")) {
					mIsCalibrated = loadCalibrationFile(_json["calibFile"]);
				}
				else {
					mIsCalibrated = false;
				}
			}
			else {
				std::cout << "[STEREO CAMERA][CUSTOM] Internal error configuring ZED camera" << std::endl;
				return false;
			}
#else
			mCameraLeft = new VideoCapture(int(_json["indexZed"]));
			if(int(_json.contains("resolution"))){
				#ifdef HAS_OPENCV_3
				int cap_prop_width = CV_CAP_PROP_FRAME_WIDTH;
				int cap_prop_height = CV_CAP_PROP_FRAME_HEIGHT;
				#elif HAS_OPENCV_4
				int cap_prop_width = cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH;
				int cap_prop_height = cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT;
				#endif

				mCameraLeft->set(cap_prop_width, ((int) _json["resolution"]["width"])*2);
				mCameraLeft->set(cap_prop_height, (int) _json["resolution"]["height"]);
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
				if(mCameraLeft->get(cap_prop_width) !=  (((int) _json["resolution"]["width"])*2)){
					std::cout << "[STEREO CAMERA][CUSTOM]  Couldn't set camera resolution " << 
								(int) _json["resolution"]["width"] << ". Current resolution is " << 
								mCameraLeft->get(cap_prop_width)/2 << std::endl;
					return false;
				}
			}
			mType = eDeviceType::zed;
			if (!mCameraLeft->isOpened())
				return false;

			if (_json.contains("calibFile")) {
				mIsCalibrated = loadCalibrationFile(_json["calibFile"]);
			}
			else {
				mIsCalibrated = false;
				return true;
			}
#endif	// HAS_ZED_SDK
		}
		else {
			std::cout << "[STEREO CAMERA][CUSTOM] That device is not currently supported." << std::endl;
			return false;
		}

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::loadCalibrationFile(const std::string &_filePath) {
		cv::FileStorage fs(_filePath, cv::FileStorage::READ);
		if (!fs.isOpened()) {
			return false;
		}

		fs["MatrixLeft"] >> mMatrixLeft;					if (mMatrixLeft.rows == 0) return false;
		fs["DistCoeffsLeft"] >> mCoefLeft;					if (mCoefLeft.rows == 0) return false;
		fs["MatrixRight"] >> mMatrixRight;					if (mMatrixRight.rows == 0) return false;
		fs["DistCoeffsRight"] >> mCoefRight;				if (mCoefRight.rows == 0) return false;
		fs["Rotation"] >> mR;								if (mR.rows == 0) return false;
		fs["Translation"] >> mT;							if (mT.rows == 0) return false;
		fs["Essential"] >> mE;								if (mE.rows == 0) return false;
		fs["Fundamental"] >> mF;							if (mF.rows == 0) return false;
		fs["RectificationLeft"] >> mRectificationLeft;		if (mRectificationLeft.rows == 0) return false;
		fs["RectificationRight"] >> mRectificationRight;	if (mRectificationRight.rows == 0) return false;
		fs["ProjectionLeft"] >> mProjectionLeft;			if (mProjectionLeft.rows == 0) return false;
		fs["ProjectionLeft"] >> mProjectionRight;			if (mProjectionRight.rows == 0) return false;
		fs["DisparityToDepth"] >> mDisparityToDepth;		if (mDisparityToDepth.rows == 0) return false;


		// Make sure that all the matrixes has the proper format to avoid crashes.
		mMatrixLeft.convertTo(mMatrixLeft, CV_64F);
		mMatrixRight.convertTo(mMatrixRight, CV_64F);
		mCoefLeft.convertTo(mCoefLeft, CV_64F);
		mCoefRight.convertTo(mCoefRight, CV_64F);
		mRectificationLeft.convertTo(mRectificationLeft, CV_64F);
		mRectificationRight.convertTo(mRectificationRight, CV_64F);
		mProjectionLeft.convertTo(mProjectionLeft, CV_64F);
		mProjectionRight.convertTo(mProjectionRight, CV_64F);
		mDisparityToDepth.convertTo(mDisparityToDepth, CV_64F);
		mR.convertTo(mR, CV_64F);
		mT.convertTo(mT, CV_64F);
		mE.convertTo(mE, CV_64F);
		mF.convertTo(mF, CV_64F);

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::decodeCloudType(const cjson::Json & _json) {
		if (!_json.contains("type"))
			return true;

		if (_json["type"] == "sparse") {
			mCloudType = eCloudType::Sparse;
			if (_json.contains("sparse")) {
				decodeDetector(_json["sparse"]["detector"]);
				decodeDescriptor(_json["sparse"]["descriptor"]);
				decodeMatcher(_json["sparse"]["matcher"]);
			}
			else {
				std::cout << "[STEREO CAMERA][CUSTOM] Specified cloud sparse but not defined method" << std::endl;
				return false;
			}
		}
		else if (_json["type"] == "dense") {
			mCloudType = eCloudType::Dense;
			if (_json.contains("dense")) {
				decodeDisparityAlgorithm(_json["dense"]["disparity"]);
			}
			else {
				std::cout << "[STEREO CAMERA][CUSTOM] Specified cloud dense but not defined method" << std::endl;
				return false;
			}

		}
		else if (_json["type"] == "null") {
			std::cout << "[STEREO CAMERA][CUSTOM] Camera configured without any algorithm for cloud generation" << std::endl;
			return true;
		}
		else {
			std::cout << "[STEREO CAMERAS][CUSTOM] Not recognize type of cloud in configuration file" << std::endl;
			return true;
		}

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void StereoCameraCustom::decodeDetector(const cjson::Json &_json) {
		if (_json["type"] == "ShiTomasi") {
			mFeatureDetector = eFeatureDetector::ShiTomasi;
			mDetectorParams = _json["params"];
		}
		else if (_json["type"] == "SIFT") {
			std::cout << "[STEREO CAMERAS][CUSTOM] SIFT Feature detector is not currently implemented." << std::endl;
			mFeatureDetector = eFeatureDetector::Null;
		}
		else if (_json["type"] == "SURF") {
			std::cout << "[STEREO CAMERAS][CUSTOM] SURF Feature detector is not currently implemented." << std::endl;
			mFeatureDetector = eFeatureDetector::Null;
		}
		else if (_json["type"] == "FAST") {
			std::cout << "[STEREO CAMERAS][CUSTOM] FAST Feature detector is not currently implemented." << std::endl;
			mFeatureDetector = eFeatureDetector::Null;
		}
		else if (_json["type"] == "ORB") {
			std::cout << "[STEREO CAMERAS][CUSTOM] ORB Feature detector is not currently implemented." << std::endl;
			mFeatureDetector = eFeatureDetector::Null;
		}
		else {
			std::cout << "[STEREO CAMERAS][CUSTOM] Undefined feature detector" << std::endl;
			mFeatureDetector = eFeatureDetector::Null;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	void StereoCameraCustom::decodeDescriptor(const cjson::Json &_json) {
		if (_json["type"] == "SIFT") {
			std::cout << "[STEREO CAMERAS][CUSTOM] SIFT feature descriptor is not implemented yet." << std::endl;
			mFeatureDescriptor = eFeatureDescriptor::Null;
		}
		else if (_json["type"] == "SURF") {
			std::cout << "[STEREO CAMERAS][CUSTOM] SURF feature descriptor is not implemented yet." << std::endl;
			mFeatureDescriptor = eFeatureDescriptor::Null;
		}
		else if (_json["type"] == "BRISK") {
			std::cout << "[STEREO CAMERAS][CUSTOM] BRISK feature descriptor is not implemented yet." << std::endl;
			mFeatureDescriptor = eFeatureDescriptor::Null;
		}
		else {
			std::cout << "[STEREO CAMERAS][CUSTOM] Undefined feature detector" << std::endl;
			mFeatureDescriptor = eFeatureDescriptor::Null;
		}

	}

	//---------------------------------------------------------------------------------------------------------------------
	void StereoCameraCustom::decodeMatcher(const cjson::Json &_json) {
		if (_json["type"] == "template") {
			mMatchingAlgorithm = eMatchingAlgorithm::TemplateMatching;
		}
		else if (_json["type"] == "brute") {
			std::cout << "[STEREO CAMERAS][CUSTOM] Force brute matcher not implemented" << std::endl;
			mMatchingAlgorithm = eMatchingAlgorithm::Null;
		}
		else if (_json["type"] == "flann") {
			std::cout << "[STEREO CAMERAS][CUSTOM] Flann based matcher not implemented" << std::endl;
			mMatchingAlgorithm = eMatchingAlgorithm::Null;
		}
		else {
			std::cout << "[STEREO CAMERAS][CUSTOM] Undefined feature matcher" << std::endl;
			mMatchingAlgorithm = eMatchingAlgorithm::Null;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	void StereoCameraCustom::decodeDisparityAlgorithm(const cjson::Json &_json) {
		if (_json["algorithm"] == "BM") {
			mDepthAlgorithm = eDepthAlgorithm::BM;
			mDisparityParams = _json["params"];
		}
		else if (_json["algorithm"] == "SGBM") {
			mDepthAlgorithm = eDepthAlgorithm::SGBM;
			mDisparityParams = _json["params"];
		}
		else if (_json["algorithm"] == "elas") {
			mDepthAlgorithm = eDepthAlgorithm::elas;
		}
		else {
			std::cout << "[STEREO CAMERAS][CUSTOM] Undefined depth algorithm" << std::endl;
			mDepthAlgorithm = eDepthAlgorithm::Null;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::disparityLibelas(cv::Mat & _depth) {
#ifdef USE_LIBELAS
		cv::Mat left, right;
		((StereoCameraCustom*)this)->rgb(left, right);

		#ifdef HAS_OPENCV_3
		cv::cvtColor(left, left, CV_BGR2GRAY);
		cv::cvtColor(right, right, CV_BGR2GRAY);
		#elif HAS_OPENCV_4
		cv::cvtColor(left, left, cv::ColorConversionCodes::COLOR_BGR2GRAY);
		cv::cvtColor(right, right, cv::ColorConversionCodes::COLOR_BGR2GRAY);
		#endif

		int width = left.cols;
		int height = left.rows;

		const int32_t dims[3] = { width,height,width }; // bytes per line = width
		float* D1_data = (float*)malloc(width*height * sizeof(float));
		float* D2_data = (float*)malloc(width*height * sizeof(float));


		// 666 TODO, choose between long and short range parameters.
		elas::Elas::parameters params;
		params = elas::Elas::parameters(elas::Elas::MIDDLEBURY);
		params.disp_min = 100;
		params.disp_max = 450;
		params.support_threshold = 0.95;
		params.support_texture = 10;
		params.candidate_stepsize = 5;
		params.incon_window_size = 5;
		params.incon_threshold = 5;
		params.incon_min_support = 5;
		params.add_corners = 0;
		params.grid_size = 20;
		params.beta = 0.02;
		params.gamma = 3;
		params.sigma = 1;
		params.sradius = 2;
		params.match_texture = 1;
		params.lr_threshold = 2;
		params.speckle_sim_threshold = 1;
		params.speckle_size = 200;
		params.ipol_gap_width = 300;
		params.filter_median = 0;
		params.filter_adaptive_mean = 1;
		params.postprocess_only_left = 1;
		params.subsampling = 0;

		elas::Elas elas(params);

		elas.process(left.data, right.data, D1_data, D2_data, dims);

		_depth = cv::Mat(height, width, CV_32F, D1_data);
		_depth.convertTo(_depth, CV_64F);

		return true;
#else
		return false;
#endif
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::disparityCvBm(cv::Mat &_depth) {
        /*cv::StereoBM matcher(cv::StereoBM::BASIC_PRESET, mDisparityParams["numDisp"], mDisparityParams["sadWindow"]);

		cv::Mat left, right;
		((StereoCameraCustom*)this)->rgb(left, right);
		cv::cvtColor(left, left, CV_BGR2GRAY);
		cv::cvtColor(right, right, CV_BGR2GRAY);

        matcher(left, right, _depth, CV_32F);*/

        return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::disparityCvSgbm(cv::Mat &_depth) {
        /*cv::StereoSGBM matcher;

		matcher.minDisparity = mDisparityParams["minDisp"];
		matcher.numberOfDisparities = mDisparityParams["numDisp"];
		matcher.SADWindowSize = mDisparityParams["sadWindow"];
		matcher.P1 = mDisparityParams["P1"];
		matcher.P2 = mDisparityParams["P2"];
		matcher.disp12MaxDiff = mDisparityParams["disp12MaxDiff"];
		matcher.preFilterCap = mDisparityParams["preFilterCap"];
		matcher.uniquenessRatio = mDisparityParams["uniquenessRatio"];
		matcher.speckleWindowSize = mDisparityParams["speckleWindowSize"];
		matcher.speckleRange = mDisparityParams["speckleRange"];
		matcher.fullDP = (bool)mDisparityParams["fullDP"];

		cv::Mat left, right;
		((StereoCameraCustom*)this)->rgb(left, right);
		cv::cvtColor(left, left, CV_BGR2GRAY);
		cv::cvtColor(right, right, CV_BGR2GRAY);

		matcher(left, right, _depth);
        _depth.convertTo(_depth, CV_32F, 1.0 / 16.0);*/

        return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeCloudDense(pcl::PointCloud<pcl::PointXYZ>::Ptr & _cloud) {
		cv::Mat depth;
		((StereoCameraCustom*)this)->depth(depth);

		cv::Mat_<double> vecTmp(4, 1);
		for (int i = 0; i < depth.rows; i++) {
			for (int j = 0; j < depth.cols; j++) {
				vecTmp(2) = depth.at<float>(i, j);
				if (vecTmp(2) == 0.0)
					continue;

				vecTmp(0) = j; vecTmp(1) = i; vecTmp(3) = 1;

				vecTmp = mDisparityToDepth*vecTmp;
				vecTmp /= vecTmp(3);
				_cloud->push_back(PointXYZ(vecTmp(0), vecTmp(1), vecTmp(2)));

			}
		}

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeCloudDense(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _cloud) {
		cv::Mat depth, left, right;
		((StereoCameraCustom*)this)->rgb(left, right);
		((StereoCameraCustom*)this)->depth(depth);

		cv::Mat_<double> vecTmp(4, 1);
		for (int i = 0; i < depth.rows; i++) {
			for (int j = 0; j < depth.cols; j++) {
				vecTmp(2) = depth.at<float>(i, j);
				if (vecTmp(2) == 0.0)
					continue;

				vecTmp(0) = j; vecTmp(1) = i; vecTmp(3) = 1;
				vecTmp = mDisparityToDepth*vecTmp;
				vecTmp /= vecTmp(3);

				pcl::PointXYZRGB point;
				point.z = vecTmp(0);
				point.x = vecTmp(1);
				point.y = vecTmp(2);

				if (left.channels() == 3) {
					point.r = left.at<cv::Vec3b>(i, j)[2];
					point.g = left.at<cv::Vec3b>(i, j)[1];
					point.b = left.at<cv::Vec3b>(i, j)[0];
				}
				else if (left.channels() == 4) {
					point.r = left.at<cv::Vec4b>(i, j)[2];
					point.g = left.at<cv::Vec4b>(i, j)[1];
					point.b = left.at<cv::Vec4b>(i, j)[0];
				}
				else {
					std::cout << "Camera do not provide colors!" << std::endl;
				}

				_cloud->push_back(point);
			}
		}

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeCloudDense(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _cloud) {
		std::cout << "[STEREO CAMERA][CUSTOM] Dense XYZRGBNormal cloud not currently implemented" << std::endl;
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeCloudSparse(pcl::PointCloud<pcl::PointXYZ>::Ptr& _cloud) {
		Mat left, right;
		((StereoCameraCustom*)this)->rgb(left, right);
		
		#ifdef HAS_OPENCV_3
		cv::cvtColor(left, left, CV_BGR2GRAY);
		cv::cvtColor(right, right, CV_BGR2GRAY);
		#elif HAS_OPENCV_4
		cv::cvtColor(left, left, cv::ColorConversionCodes::COLOR_BGR2GRAY);
		cv::cvtColor(right, right, cv::ColorConversionCodes::COLOR_BGR2GRAY);
		#endif


		// Compute keypoint only in first image
		vector<Point2i> keypoints;
		if (!computeFeatures(left, keypoints))
			return false;

		std::cout << "--> STEREO: Features computed in frame1: " << keypoints.size() << std::endl;

		// Compute projection of epipolar lines into second image.
		std::vector<cv::Vec3f> epilines;
		computeEpipoarLines(keypoints, epilines);

		// For each epipolar line calculate equivalent feature by template matching.
		const int squareSize = 11;
		Rect secureRegion(squareSize / 2 + 1,
			squareSize / 2 + 1,
			right.cols - 2 * (squareSize / 2 + 1),
			right.rows - 2 * (squareSize / 2 + 1));
		Rect validLeft(0, 0, mLeftFrame.cols, mLeftFrame.rows);
		validLeft &= secureRegion;
		Rect validRight(0, 0, mLeftFrame.cols, mLeftFrame.rows);
        validRight &= secureRegion;


		const unsigned cNumProcs = 8;
		vector<vector<Point2i>> vpoints1(cNumProcs), vpoints2(cNumProcs);

		std::pair<int, int> disparityRange(40, 400);
		double maxTemplateScore = 0.01;
		// Match features using ParallelFeatureMatcher Class
		parallel_for_(Range(0, cNumProcs), ParallelFeatureMatcher(left, right, keypoints, epilines, disparityRange, squareSize, maxTemplateScore, vpoints1, vpoints2, validLeft, validRight));

		vector<Point2i> points1, points2;

		for (vector<Point2i> v : vpoints1) {
			points1.insert(points1.end(), v.begin(), v.end());
		}

		for (vector<Point2i> v : vpoints2) {
			points2.insert(points2.end(), v.begin(), v.end());
		}

		std::cout << "--> STEREO: Features matched: " << points1.size() << std::endl;
		// Triangulate points using features in both images.
		vector<Point3f> points3d = triangulate(points1, points2);
		// Filter points using reprojection.

		for (auto point : points3d) {
			//if (point.z > 0.2 && point.z < 1.0) {
			PointXYZ pclPoint(point.x, point.y, point.z);
			_cloud->push_back(pclPoint);
			//}
		}
		std::cout << "--> STEREO: Points in the selected range: " << _cloud->size() << std::endl;

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeCloudSparse(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _cloud) {
		Mat left, right, leftColor, rightColor;
		((StereoCameraCustom*)this)->rgb(leftColor, rightColor);

		#ifdef HAS_OPENCV_3
		cv::cvtColor(leftColor, left, CV_BGR2GRAY);
		cv::cvtColor(rightColor, right, CV_BGR2GRAY);
		#elif HAS_OPENCV_4
		cv::cvtColor(leftColor, left, cv::ColorConversionCodes::COLOR_BGR2GRAY);
		cv::cvtColor(rightColor, right, cv::ColorConversionCodes::COLOR_BGR2GRAY);
		#endif

		// Compute keypoint only in first image
		vector<Point2i> keypoints;
		if (!computeFeatures(left, keypoints))
			return false;

		std::cout << "--> STEREO: Features computed in frame1: " << keypoints.size() << std::endl;

		// Compute projection of epipolar lines into second image.
		std::vector<cv::Vec3f> epilines;
		computeEpipoarLines(keypoints, epilines);

		// For each epipolar line calculate equivalent feature by template matching.
		const int squareSize = 11;
		Rect secureRegion(squareSize / 2 + 1,
			squareSize / 2 + 1,
			right.cols - 2 * (squareSize / 2 + 1),
			right.rows - 2 * (squareSize / 2 + 1));
		Rect validLeft(50, 10, 470, 460);
		validLeft &= secureRegion;
		Rect validRight(20, 10, 470, 460);
		validRight &= secureRegion;


		const unsigned cNumProcs = 8;
		vector<vector<Point2i>> vpoints1(cNumProcs), vpoints2(cNumProcs);

		std::pair<int, int> disparityRange(40, 400);
		double maxTemplateScore = 0.01;
		// Match features using ParallelFeatureMatcher Class
		parallel_for_(Range(0, cNumProcs), ParallelFeatureMatcher(left, right, keypoints, epilines, disparityRange, squareSize, maxTemplateScore, vpoints1, vpoints2, validLeft, validRight));

		vector<Point2i> points1, points2;

		for (vector<Point2i> v : vpoints1) {
			points1.insert(points1.end(), v.begin(), v.end());
		}

		for (vector<Point2i> v : vpoints2) {
			points2.insert(points2.end(), v.begin(), v.end());
		}

		std::cout << "--> STEREO: Features matched: " << points1.size() << std::endl;
		// Triangulate points using features in both images.
		vector<Point3f> points3d = triangulate(points1, points2);
		// Filter points using reprojection.

		for (unsigned i = 0; i < points3d.size(); i++) {
			auto point = points3d[i];
			PointXYZRGB pclPoint;
			pclPoint.x = point.x;
			pclPoint.y = point.y;
			pclPoint.z = point.z;

			if (leftColor.channels() == 3) {
				pclPoint.r = leftColor.at<cv::Vec3b>(points1[i].y, points1[i].x)[2];
				pclPoint.g = leftColor.at<cv::Vec3b>(points1[i].y, points1[i].x)[1];
				pclPoint.b = leftColor.at<cv::Vec3b>(points1[i].y, points1[i].x)[0];
			}
			else if (leftColor.channels() == 4) {
				pclPoint.r = leftColor.at<cv::Vec4b>(points1[i].y, points1[i].x)[2];
				pclPoint.g = leftColor.at<cv::Vec4b>(points1[i].y, points1[i].x)[1];
				pclPoint.b = leftColor.at<cv::Vec4b>(points1[i].y, points1[i].x)[0];
			}
			else {
				std::cout << "Camera do not provide colors!" << std::endl;
			}

			_cloud->push_back(pclPoint);

		}
		std::cout << "--> STEREO: Points in the selected range: " << _cloud->size() << std::endl;

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeCloudSparse(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _cloud) {
		std::cout << "[STEREO CAMERA][CUSTOM] Sparse XYZRGBNormal cloud not currently implemented" << std::endl;
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeFeatures(const Mat &_frame, vector<Point2i> &_features) {
		switch (mFeatureDetector) {
		case eFeatureDetector::FAST:
			break;
		case eFeatureDetector::ORB:
			break;
		case eFeatureDetector::SIFT:
			break;
		case eFeatureDetector::SURF:
			break;
		case eFeatureDetector::ShiTomasi:
			goodFeaturesToTrack(_frame, _features, mDetectorParams["nFeatures"], mDetectorParams["qualityLevel"], mDetectorParams["minDist"]);
			break;
		default:
			std::cout << "[STEREO CAMERA][CUSTOM] Not feature detector configured" << std::endl;
			return false;
			break;
		}

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void StereoCameraCustom::computeEpipoarLines(const vector<Point2i> &_points, vector<Vec3f> &_epilines) {
		vector<Point2f> points;
		for (auto point : _points) {
			points.push_back(Point2f(point.x, point.y));
		}

		computeCorrespondEpilines(points, 1, mF, _epilines);
	}

	//---------------------------------------------------------------------------------------------------------------------
	vector<Point3f> StereoCameraCustom::triangulate(const vector<Point2i> &_points1, const vector<Point2i> &_points2) {
		Mat pnts3D(4, _points1.size(), CV_64F);
		Mat cam1pnts(2, _points1.size(), CV_64F);
		Mat cam2pnts(2, _points1.size(), CV_64F);

		for (unsigned i = 0; i < _points1.size(); i++) {
			cam1pnts.at<double>(0, i) = _points1[i].x;
			cam1pnts.at<double>(1, i) = _points1[i].y;
			cam2pnts.at<double>(0, i) = _points2[i].x;
			cam2pnts.at<double>(1, i) = _points2[i].y;
		}

		Mat R1, R2, P1, P2, Q;
		stereoRectify(mMatrixLeft, mCoefLeft, mMatrixRight, mCoefRight, mLeftFrame.size(), mR, mT, R1, R2, P1, P2, Q);

		triangulatePoints(P1, P2, cam1pnts, cam2pnts, pnts3D);

		vector<Point3f> points3d;
		for (int i = 0; i < pnts3D.cols; i++) {
			float w = (float)pnts3D.at<double>(3, i);
			float x = (float)pnts3D.at<double>(0, i) / w;
			float y = (float)pnts3D.at<double>(1, i) / w;
			float z = (float)pnts3D.at<double>(2, i) / w;
			points3d.push_back(Point3f(x, y, z));
		}

		return points3d;
	}
}	//	namespace mico 
