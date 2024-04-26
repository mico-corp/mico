//-----------------------------------------------------------------------------
//  Cameras wrapper MICO plugin
//-----------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//-----------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to
//  deal in the Software without restriction, including without limitation the
//  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
//  sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//  IN THE SOFTWARE.
//-----------------------------------------------------------------------------

#ifndef MICO_BASE_VISION_STEREOCAMERAS_StereoCameraRosBag_H_
#define MICO_BASE_VISION_STEREOCAMERAS_StereoCameraRosBag_H_

#include <mico/cameras/StereoCamera.h>
#ifdef RGBDTOOLS_USE_ROS
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#endif

#include <mutex>
#include <thread>

namespace mico {
/// Wrapper for generic ROS camera
class StereoCameraRosBag : public StereoCamera {
public: // Public interface
  /// \brief Initialize the camera using a config file. The coordinate system
  /// for the 3d is always on the color camera. Config file must have following
  /// structure.
  ///
  /// \code
  ///     {
  ///			"left":"Topic to left camera, if camera is structural light,
  ///then
  /// left is the only color camera", 			"right":"In case of stereo camera,
  /// topic to right camera", 			"depth":"Topic to depth camera",
  /// "depth_encoding":"16U|32F", 			"calibration":
  ///				{
  ///					"type":"topic|file",
  ///					"config":
  ///						{
  ///							"topic":"In case of topic type camera info, topic
  ///to camera info",
  /// "file_path":"In case of file type, path to camera calibration file"
  ///						}
  ///				}m
  ///			"cloud":"Topic to cloud"
  ///     }
  /// \endcode
  ///
  /// \param _filePath: path to the file.
  bool init(const cjson::Json &_json = "");

  /// \brief get the rgb frame and fill only the left image.
  bool rgb(cv::Mat &_left, cv::Mat &_right);

  /// \brief get the depth frame generated by the IR sensors.
  bool depth(cv::Mat &_depth);

  /// \brief Grab current data from camera to make sure that is synchronized.
  bool grab();

  /// \brief Get a new point cloud from the camera with only spatial
  /// information. \param _cloud: reference to a container for the point cloud.
  bool cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud);

  /// \brief Get a new point cloud from the camera with spatial and RGB
  /// information. \param _cloud: reference to a container for the point cloud.
  bool cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud);

  /// \brief Get a new point cloud from the camera with spatial, surface normals
  /// and RGB information. \param _cloud: reference to a container for the point
  /// cloud.
  bool cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud);

  /// \brief Get a new point cloud from the camera with spatial information and
  /// surface normals. \param _cloud: reference to a container for the point
  /// cloud.
  bool cloud(pcl::PointCloud<pcl::PointNormal> &_cloud);

  /// \brief templatized method to define the interface for retrieving a new
  /// point cloud from the camera with spatial, surface normals and RGB
  /// information (custom types out of PCL). \param _cloud: reference to a
  /// container for the point cloud.
  template <typename PointType_>
  bool cloud(pcl::PointCloud<PointType_> &_cloud);

  /// \brief get the calibration matrices of the left camera in opencv format.
  /// Matrices are CV_32F.
  bool leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients);

  /// \brief get the calibration matrices of the depth (IR) camera in opencv
  /// format. Matrices are CV_32F.
  bool rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients);

  /// \brief get the extrinsic matrices, i.e., transformation from left to depth
  /// (IR) camera. Matrices are CV_32F.
  bool extrinsic(cv::Mat &_rotation, cv::Mat &_translation);

  /// \brief get the extrinsic matrices, i.e., transformation from left to depth
  /// (IR) camera.
  bool extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation);

  /// \brief get disparty-to-depth parameter typical from RGB-D devices.
  bool disparityToDepthParam(double &_dispToDepth);

  bool colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point);

private:
  template <typename PointType_>
  bool setOrganizedAndDense(pcl::PointCloud<PointType_> &_cloud);

private: // Private members
  cjson::Json mConfig;
  cv::Mat mLastRGB, mRight, mLastDepthInColor;

#ifdef RGBDTOOLS_USE_ROS
  rosbag::Bag mBag;
  rosbag::View *leftView, *rightView, *depthView;
  rosbag::View::iterator leftIt, rightIt, depthIt;
#endif
  bool mHasLeft = false, mHasRight = false, mHasDepth = false;

  bool mHasCalibration = false;
  cv::Mat mMatrixLeft, mDistCoefLeft, mMatrixRight, mDistCoefRight, mRot,
      mTrans;
  double mDispToDepth;

}; //	class StereoCameraRosBag

template <typename PointType_>
inline bool
StereoCameraRosBag::setOrganizedAndDense(pcl::PointCloud<PointType_> &_cloud) {
  _cloud.is_dense =
      true; // 666 TODO: cant set to true if wrong points are set to NaN.
  _cloud.width = mLastRGB.cols;
  _cloud.height = mLastRGB.rows;
  return true;
}

} // namespace mico

#include <mico/cameras/StereoCameras/StereoCameraRosBag.inl>

#endif // RGBDSLAM_VISION_STEREOCAMERAS_StereoCameraRosBag_H_
