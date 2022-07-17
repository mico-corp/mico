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


#if defined(_MSC_VER)
#  pragma warning(push)
#  pragma warning(disable: 4702) // warning C4702: unreachable code
#endif

#include <mico/ml/flow/BlockFaceFeatures.h>
#include <flow/Outpipe.h>

#include <cmath>
#include <sstream>

namespace mico{
    namespace ml{
        BlockFaceFeatures::BlockFaceFeatures(){

            try{
                dlib::deserialize((flow::Persistency::resourceDir() / "ml" / "dlib_face_recognition_resnet_model_v1.dat").string()) >> featureDetector_;
            }catch(std::exception &_e){
                std::cerr << "[WARNING] Failed to read YOLO parameters, cant use YOLO ML block." << _e.what() << std::endl;
            }

            
            createPipe<std::vector<Detection>>("descriptor");

            createPolicy({  flow::makeInput<cv::Mat>("face") });

            registerCallback(   {"face"}, 
                                &BlockFaceFeatures::policyCallback,
                                this
            );
        }

        BlockFaceFeatures::~BlockFaceFeatures() {
            
        }

        void BlockFaceFeatures::policyCallback(cv::Mat _image) {
            if (getPipe("descriptor")->registrations()) {
                cv::Mat frame = _image.clone();
                if (frame.rows != 0) {
                    // Resize ro 150x150. Should normalize too but I am tired now 666 TODO
                    cv::resize(frame, frame, cv::Size(150, 150));

                    dlib::matrix<dlib::rgb_pixel> faceImg;
                    IplImage iplImg = cvIplImage(frame);
                    dlib::cv_image<dlib::bgr_pixel> cimg(&iplImg);
                    dlib::assign_image(faceImg, cimg);

                    std::vector<dlib::matrix<dlib::rgb_pixel>> faces = { faceImg };
                    std::vector<dlib::matrix<float, 0, 1>> faceDescriptors = featureDetector_(faces);

                    std::vector<float> features(128);
                    memcpy(features.data(), faceDescriptors[0].begin(), sizeof(float) * 128);

                    if (getPipe("descriptor")->registrations()) getPipe("descriptor")->flush(features);
                }
            }
        }
    }
}


#if defined(_MSC_VER)
#  pragma warning(pop)
#endif