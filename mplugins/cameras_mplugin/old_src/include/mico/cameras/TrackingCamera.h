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

#ifndef MICO_BASE_VISION_TRACKING_CAMERA_H_
#define MICO_BASE_VISION_TRACKING_CAMERA_H_

#ifdef ENABLE_LIBREALSENSE_V2

#include <mico/cameras/cjson/json.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <string>
#include <iostream>
#include <thread>
#include <mutex>

#include <librealsense2/rs.hpp>

namespace mico{
    
    class TrackingCamera {
    public:
        /// \param _filePath: path to the file.
		bool init(const cjson::Json &_json = "");

		/// \brief Grab current data from camera to make sure that is synchronized.
		bool grab();

        /// \brief get the mono fisheye frame.
		bool fisheye(cv::Mat &_fisheye);
        
        /// \brief get the t265 pose.
		bool pose(Eigen::Matrix4f &_pose);

        /// \brief reset the camera pose.
        bool reset();

    private:
        int deviceId_;
        cjson::Json configFile_;
        
        rs2::context rsContext_;
        rs2::device rsDevice_;
        rs2::pipeline rsPipeline_;
        rs2::pipeline_profile rsPipelineProfile_;
        rs2::config rsConfig_;

        rs2_intrinsics fisheyeIntrinsics_;

        cv::Mat lastLeftFisheye_;
        Eigen::Matrix4f lastPose_;
        bool hasFisheye_ , hasPose_;

        int leftId_  = 1;
        int rightId_ = 2;

        std::mutex pipelineLock_;
    };
}

#endif

#endif