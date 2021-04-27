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


// #include <mico/cameras/StereoCameras/StereoCameraRealSense.h>
// #include <mico/cameras/StereoCameras/StereoCameraVirtual.h>

namespace mico {
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool StereoCamera::cloud(pcl::PointCloud<PointType_> &_cloud) {
        // 666 TODO: code implementation to avoid cast in main code and encapsulate behaviour in the library
        // Based on Andrei Alexandrescu  multimethods guide http://www.icodeguru.com/CPP/ModernCppDesign/0201704315_ch11.html
        //if(StereoCameraRealSense * camera = dynamic_cast<StereoCameraRealSense *>(this)){
        //    camera->cloud(_cloud);
        //    return true;
        //}else if(StereoCameraVirtual * camera = dynamic_cast<StereoCameraVirtual *>(this)){
        //    camera->cloud(_cloud);
        //    return true;
        //}else{
            std::cerr << "[STEREOCAMERA] cloud method for custom types not implemented for given point type in used StereoCamera." << std::endl;
            return false;
        //}
    }
}
