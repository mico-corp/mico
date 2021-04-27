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

#ifdef ENABLE_LIBREALSENSE_V2

#include <mico/cameras/TrackingCamera.h>
namespace mico {

	bool TrackingCamera::init(const cjson::Json & _json){
        configFile_ = _json;

        if(configFile_.contains("deviceId")){
            deviceId_ = configFile_["deviceId"];
        }
        
        // 666 
        // https://github.com/IntelRealSense/librealsense/issues/3434
        // 
        // rsContext_ = rs2::context();
        // auto list = rsContext_.query_devices();
        // std::this_thread::sleep_for( std::chrono::seconds(1) );
        // rsContext_.unload_tracking_module();
		// if (list.size() == 0) {
		// 	std::cout << "[REALSENSE] There's no any compatible device connected." << std::endl;
		// 	return false;
		// }
        // rsDevice_ = list[deviceId_];

        // std::cout << "[REALSENSE] Using device "<< deviceId_ <<", an "<< rsDevice_.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << std::endl;
        // std::cout << "[REALSENSE] Serial number: " << rsDevice_.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        // std::cout << "[REALSENSE] Firmware version: " << rsDevice_.get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION)<< std::endl;

        rsConfig_.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        rsConfig_.enable_stream(RS2_STREAM_FISHEYE, leftId_ , RS2_FORMAT_Y8);
        rsConfig_.enable_stream(RS2_STREAM_FISHEYE, rightId_, RS2_FORMAT_Y8);
        pipelineLock_.lock();
        rsPipeline_ = rs2::pipeline();
        rsPipelineProfile_ = rsPipeline_.start(rsConfig_);
        pipelineLock_.unlock();

        auto fisheyeStream = rsPipelineProfile_.get_stream(RS2_STREAM_FISHEYE).as<rs2::video_stream_profile>();
        fisheyeIntrinsics_ = fisheyeStream.get_intrinsics();        

        return true;            
    }

    bool TrackingCamera::grab(){
        pipelineLock_.lock();
        rs2::frameset frames = rsPipeline_.wait_for_frames();
        pipelineLock_.unlock();

        rs2::frame framePose = frames.first(RS2_STREAM_POSE);
        auto poseData = framePose.as<rs2::pose_frame>().get_pose_data();
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3,1>(0,3) = Eigen::Vector3f(poseData.translation.x , poseData.translation.y , poseData.translation.z);
        pose.block<3,3>(0,0) = Eigen::Quaternionf(poseData.rotation.w , poseData.rotation.x , poseData.rotation.y , poseData.rotation.z).normalized().toRotationMatrix();
        lastPose_ = pose;
        hasPose_ = true;

        rs2::frame frameFisheye = frames.get_fisheye_frame(leftId_);
        lastLeftFisheye_ = cv::Mat(cv::Size(fisheyeIntrinsics_.width, fisheyeIntrinsics_.height), CV_8UC1, (void*)frameFisheye.get_data(), cv::Mat::AUTO_STEP);
		hasFisheye_ = true;

        return true;
    }

    bool TrackingCamera::fisheye(cv::Mat &_fisheye){
        lastLeftFisheye_.copyTo(_fisheye);

        return hasFisheye_;
    }

    bool TrackingCamera::pose(Eigen::Matrix4f &_pose){
        _pose = lastPose_;

        return hasPose_;
    }

    bool TrackingCamera::reset(){
        pipelineLock_.lock();

        rsPipeline_.stop();
        std::this_thread::sleep_for( std::chrono::seconds(1) );
        rsPipeline_.start(rsConfig_);
        
        pipelineLock_.unlock();
        return true;
    }
    
}

#endif