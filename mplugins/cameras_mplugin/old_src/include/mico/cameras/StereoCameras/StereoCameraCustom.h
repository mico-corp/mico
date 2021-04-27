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


#ifndef MICO_BASE_STEREOCAMERAS_STEREOCAMERACUSTOM_H_
#define MICO_BASE_STEREOCAMERAS_STEREOCAMERACUSTOM_H_


#include <mico/cameras/StereoCamera.h>
#include <opencv2/opencv.hpp>

#ifdef HAS_ZED_SDK
#include <zed/Camera.hpp>
#endif

namespace mico {
	/// This class is used to integrate with the system custom stereo cameras.
	class StereoCameraCustom : public StereoCamera {
	public:		// Public interface
				/// Default constructor
		StereoCameraCustom();

		/// Default destructor of class. Ensure that cameras are properly detached.
		~StereoCameraCustom();

		/// \brief Initialize the camera using a config file in json.
		/// Config file must have following structure.
		///
		/// \code
		///     {
		///         "device":
		///             {
		///                 "type":"opencv|zed",                        // One of these. Currently only opencv and zed cameras are implemented as sources
		///                 "left":"/path/to/file/template %d.jpg",     // In case of OpenCV compatible cameras, provide the source of left and right.
		///                 "left":0,									// This can be an index for the camera, a path to a video file, or a template
		///                 "right":"/path/to/video.avi",               // path to a set of images.
		///                 "right":1,
		///					"indexZed":1,								// Index of zed camera, only if using zed
		///                 "calibFile":"/path/to/calib.yml"            // This element is a path to calibration files in OpenCV format stored in yml files.
		///                                                             // This file should contain following entries: {MatrixLeft, DistCoeffsLeft, MatrixRight,
		///                                                             // DistCoeffsRight, Rotation, Translation, Essential, Fundamental, RectificationLeft,
		///																// RectificationRight, ProjectionLeft, ProjectionRight, DisparityToDepth }.
		///             },
		///         "cloud":
		///             {
		///                 "type":"null|sparse|dense",       	// In case of OpenCV cameras, it is necessary to choose between sparse cloud generation
		///                                             		// dense cloud generation.
		///                 "sparse":   						// In case of type=sparse, this need to be present.
		///                     {
		///							"detector":
		///							{
		///								"type":"SIFT|SURF|FAST|ORB|ShiTomasi",
		///									"params" :
		///								{
		///									"nFeatures":5000,
		///										"qualityLevel" : 0.01,
		///										"minDist" : 0.5
		///								}
		///							},
		///								"descriptor":
		///							{
		///								"type":"SIFT|SURF|BRISK",
		///									"params" :
		///								{
		///								}
		///							},
		///								"matcher":
		///							{
		///								"type":"brute|flann|template",
		///									"params" :
		///								{
		///								}
		///							}
		///                     },
		///					"dense":
		///						{
		///							"disparity":
		///								{
		///									"algorithm":"BM|SGBM|elas",
		///									"params:		// BM and SGM need this parameter list
		///										{
		///											"numDisp":64,			// BM and SGBM.
		///											"sadWindow":21,			// BM and SGBM.
		///											"minDisp":16,			// only SGBM.
		///											"P1":0,					// only SGBM.
		///											"P2":0,					// only SGBM.
		///											"disp12MaxDiff":0,		// only SGBM.
		///											"preFilterCap":0,		// only SGBM.
		///											"uniquenessRatio":0,	// only SGBM.
		///											"speckleWindowSize":0,	// only SGBM.
		///											"speckleRange":0,		// only SGBM.
		///											"fullDP":false			// only SGBM.
		///										}
		///								}
		///						}
		///             }
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

		/// \brief Get a new point cloud from the camera with only spatial information.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud);

		/// \brief Get a new point cloud from the camera with spatial and RGB information.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud);

		/// \brief Get a new point cloud from the camera with spatial, surface normals and RGB information.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud);

		/// \brief Get a new point cloud from the camera with spatial information and surface normals.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointNormal> &_cloud);

        bool colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point){return false;}

        /// \brief get the calibration matrices of the left camera in opencv format.  Matrices are CV_32F.
        bool leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients){
            mMatrixLeft.copyTo(_intrinsic);
            mCoefLeft.copyTo(_coefficients);
            return true;
        }

        /// \brief get the calibration matrices of the right camera in opencv format.  Matrices are CV_32F.
        bool rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients){
            mMatrixRight.copyTo(_intrinsic);
            mCoefRight.copyTo(_coefficients);
            return true;
        }

        /// \brief get the extrinsic matrices, i.e., transformation from left to right camera. Matrices are CV_32F.
        bool extrinsic(cv::Mat &_rotation, cv::Mat &_translation){
            mR.copyTo(_rotation);
            mT.copyTo(_translation);
            return true;
        }

	private:	// Private methods
				// Configuration methods
		bool configureDevice(const cjson::Json &_json);
		bool loadCalibrationFile(const std::string &_filePath);
		bool decodeCloudType(const cjson::Json &_json);
		void decodeDetector(const cjson::Json &_json);
		void decodeDescriptor(const cjson::Json &_json);
		void decodeMatcher(const cjson::Json &_json);
		void decodeDisparityAlgorithm(const cjson::Json &_json);

		// Depth algorithms
		bool disparityLibelas(cv::Mat &_depth);
		bool disparityCvBm(cv::Mat &_depth);
		bool disparityCvSgbm(cv::Mat &_depth);

		// Cloud type switcher
		bool computeCloudDense(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);
		bool computeCloudDense(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
		bool computeCloudDense(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud);

		bool computeCloudSparse(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);
		bool computeCloudSparse(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
		bool computeCloudSparse(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud);

		// Auxiliar methods
		bool computeFeatures(const cv::Mat &_frame, std::vector<cv::Point2i> &_features);
		void computeEpipoarLines(const std::vector<cv::Point2i> &_points, std::vector<cv::Vec3f> &_epilines);
		std::vector<cv::Point3f> triangulate(const std::vector<cv::Point2i> &_points1, const std::vector<cv::Point2i> &_points2);

	private:	// Members
				// Internal type to switch between functions.
		enum eDeviceType { opencv, zed };
		eDeviceType mType;

		// Calibration matrixes.
		cv::Mat mMatrixLeft, mMatrixRight, mCoefLeft, mCoefRight, mR, mT, mE, mF;
		cv::Mat mRectificationLeft, mRectificationRight, mProjectionLeft, mProjectionRight, mDisparityToDepth;
		bool mIsCalibrated = false;

		// For opencv custom stereo cameras.
		cv::VideoCapture *mCameraLeft = nullptr, *mCameraRight = nullptr;

		// For zed custom stereo camera.
#ifdef HAS_ZED_SDK
		sl::zed::Camera *mZedCamera = nullptr;
#endif	// HAS_ZED_SDK

		// Configuration for data type
		enum class eCloudType { Null, Sparse, Dense };
		enum class eFeatureDetector { Null, SIFT, SURF, ORB, FAST, ShiTomasi };
		enum class eFeatureDescriptor { Null, SIFT, SURF, ORB };
		enum class eMatchingAlgorithm { Null, Brute, Flann, TemplateMatching };
		enum class eDepthAlgorithm { Null, BM, SGBM, elas };

		eCloudType			mCloudType = eCloudType::Null;
		eFeatureDetector	mFeatureDetector = eFeatureDetector::Null;
		eFeatureDescriptor	mFeatureDescriptor = eFeatureDescriptor::Null;
		eMatchingAlgorithm	mMatchingAlgorithm = eMatchingAlgorithm::Null;
		eDepthAlgorithm		mDepthAlgorithm = eDepthAlgorithm::Null;

		cjson::Json mDetectorParams;
		cjson::Json mDisparityParams;

		// Resusable variables and variables to decide if reuse data or not.
		bool mHasRGB = false, mComputedDepth = false, mComputedCloudXYZ = false, mComputedCloudXYZRGB = false, mComputedCloudXYZRGBNormal = false, mComputedCloudNormal = false;
		cv::Mat mLeftFrame, mRightFrame, mDepth;
		pcl::PointCloud<pcl::PointXYZ>::Ptr				mCloudXYZ;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr			mCloudXYZRGB;
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr	mCloudXYZRGBNormal;
		pcl::PointCloud<pcl::PointNormal>::Ptr			mCloudXYZNormal;
	};
}	//	namespace mico 

#endif /* RGBDSLAM_STEREOCAMERAS_STEREOCAMERACUSTOM_H_ */
