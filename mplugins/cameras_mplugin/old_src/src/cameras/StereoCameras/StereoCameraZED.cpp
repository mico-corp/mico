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


#include <mico/cameras/StereoCameras/StereoCameraZED.h>

namespace mico {
	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraZed::init(const cjson::Json &_json) {
		#ifdef HAS_ZED_SDK
				if (_json["mode"] == "default") { // Default config
					mZedCamera = new sl::zed::Camera(sl::zed::ZEDResolution_mode::VGA);
					if (mZedCamera == nullptr)
						return false;

					sl::zed::ERRCODE err = mZedCamera->init(sl::zed::MODE::PERFORMANCE);
					switch (err) {
					case sl::zed::ERRCODE::SUCCESS:
						std::cout << "[STEREO CAMERA][ZED] Camera initialized" << std::endl;
						return true;
						break;
					case sl::zed::ERRCODE::NO_GPU_COMPATIBLE:
						std::cout << "[STEREO CAMERA][ZED] Error initializing Zed camera. No GPU compatibe" << std::endl;
						return false;
						break;
					case sl::zed::ERRCODE::NOT_ENOUGH_GPUMEM:
						std::cout << "[STEREO CAMERA][ZED] Error initializing Zed camera. Not enough GPU memory" << std::endl;
						return false;
						break;
					case sl::zed::ERRCODE::ZED_NOT_AVAILABLE:
						std::cout << "[STEREO CAMERA][ZED] Error initializing Zed camera. Not camera available" << std::endl;
						return false;
						break;
					case sl::zed::ERRCODE::ZED_SETTINGS_FILE_NOT_AVAILABLE:
						std::cout << "[STEREO CAMERA][ZED] Error initializing Zed camera. Not setting file found" << std::endl;
						return false;
						break;
					case sl::zed::ERRCODE::INVALID_SVO_FILE:
						std::cout << "[STEREO CAMERA][ZED] Error initializing Zed camera. Invalid SVO file" << std::endl;
						return false;
						break;
					case sl::zed::ERRCODE::RECORDER_ERROR:
						std::cout << "[STEREO CAMERA][ZED] Error initializing Zed camera. Recorder error" << std::endl;
						return false;
						break;
					default:
						std::cout << "[STEREO CAMERA][ZED] Undefined Error starting ZED camera" << std::endl;
						return false;
						break;
					}
				}
				else {  // Use config file
					return false;
				}
		#else
				return false;
		#endif
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraZed::rgb(cv::Mat &_left, cv::Mat &_right) {
		#ifdef HAS_ZED_SDK
				if (!mHasRGB) {
					sl::zed::Mat left = mZedCamera->retrieveImage(sl::zed::SIDE::LEFT);
					sl::zed::Mat right = mZedCamera->retrieveImage(sl::zed::SIDE::RIGHT);

					if (mLeftFrame.rows != left.height) {	// If image size has changed or if first iteration, prepare containers
						mLeftFrame = cv::Mat(left.height, left.width, CV_8UC4);
						mRightFrame = cv::Mat(right.height, right.width, CV_8UC4);
					}

					memcpy(mLeftFrame.data, left.data, left.width*left.height * 4 * sizeof(uchar));
					memcpy(mRightFrame.data, right.data, right.width*right.height * 4 * sizeof(uchar));

					mHasRGB = true;
				}

				_left = mLeftFrame;
				_right = mRightFrame;
				return true;
		#else
				return false;
		#endif
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraZed::depth(cv::Mat &_depth) {
		#ifdef HAS_ZED_SDK
				if (!mComputedDepth) {
					sl::zed::Mat depth = mZedCamera->retrieveMeasure(sl::zed::MEASURE::DEPTH);

					if (mDepth.rows != depth.height) {	// If changed size or first iteration, prepare container
						mDepth = cv::Mat(depth.height, depth.width, CV_32FC1);
					}
					memcpy((float*)mDepth.data, depth.data, depth.width*depth.height*sizeof(float));

					mComputedDepth = true;
				}

				_depth = mDepth;
				return true;
		#else
				return false;
		#endif
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraZed::cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud) {
		#ifdef HAS_ZED_SDK
				if (!mComputedCloudXYZ) {
					float* p_cloud = (float*)mZedCamera->retrieveMeasure(sl::zed::MEASURE::XYZ).data; // Get the pointer
					int width = mZedCamera->getImageSize().width;
					int height = mZedCamera->getImageSize().height;
					int index3 = 0;

					mCloudXYZ.clear();

					for (int i = 0; i < width*height; i++) {
						pcl::PointXYZ point;

						// Changing unit and coordinate for better visualization
						point.x = -p_cloud[index3++] * 0.001;
						point.y = -p_cloud[index3++] * 0.001;
						point.z = p_cloud[index3++] * 0.001;

						if (point.z > 0) { // Checking if it's a valid point
							mCloudXYZ.push_back(point);
						}
					}
					mComputedCloudXYZ = true;
				}
				_cloud = mCloudXYZ;
				return true;
		#else
				return false;
		#endif
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraZed::cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud) {
		#ifdef HAS_ZED_SDK
				if (!mComputedCloudXYZRGB) {
					float* p_cloud = (float*)mZedCamera->retrieveMeasure(sl::zed::MEASURE::XYZRGBA).data; // Get the pointer
					int width = mZedCamera->getImageSize().width;
					int height = mZedCamera->getImageSize().height;
					int index4 = 0;

					mCloudXYZRGB.clear();

					for (int i = 0; i < width*height; i++) {
						pcl::PointXYZRGB point;

						// Changing unit and coordinate for better visualization
						point.x = -p_cloud[index4++] * 0.001;
						point.y = -p_cloud[index4++] * 0.001;
						point.z = p_cloud[index4++] * 0.001;

						float color = p_cloud[index4++];
						// Converting color
						uint32_t color_uint = *(uint32_t*)& color;
						unsigned char* color_uchar = (unsigned char*)&color_uint;
						color_uint = ((uint32_t)color_uchar[0] << 16 | (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[2]);
						point.rgb = *reinterpret_cast<float*> (&color_uint);

						if (point.z > 0) // Checking if it's a valid point
							mCloudXYZRGB.push_back(point);
					}

					mComputedCloudXYZRGB = true;
				}

				_cloud = mCloudXYZRGB;

				return true;
		#else
				return false;
		#endif
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraZed::grab() {
		#ifdef HAS_ZED_SDK
			mHasRGB = false, mComputedDepth = false, mComputedCloudXYZ = false, mComputedCloudXYZRGB = false, mComputedCloudXYZRGBNormal = false, mComputedCloudNormal = false;
			return mZedCamera->grab();	
		#else
			return false;
		#endif
	}

}	//	namespace mico 
