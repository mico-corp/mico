// Resource from https://www.jianshu.com/p/be16847b0b74

#ifndef MICO_FLOW_MISC_PYTHON_CONVERSIONUTILS_H_
#define MICO_FLOW_MISC_PYTHON_CONVERSIONUTILS_H_

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <opencv2/opencv.hpp>

boost::python::numpy::ndarray ConvertMatToNDArray(const cv::Mat& mat);

cv::Mat ConvertNDArrayToMat(const boost::python::numpy::ndarray& ndarr);


#endif 