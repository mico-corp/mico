// Resource from https://www.jianshu.com/p/be16847b0b74

#ifndef MICO_FLOW_MISC_PYTHON_CONVERSIONUTILS_H_
#define MICO_FLOW_MISC_PYTHON_CONVERSIONUTILS_H_

#define BOOST_PYTHON_STATIC_LIB
#define BOOST_LIB_NAME "boost_numpy"
#include <boost/config/auto_link.hpp>

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <opencv2/opencv.hpp>

boost::python::numpy::ndarray ConvertMatToNDArray(const cv::Mat& mat);

cv::Mat ConvertNDArrayToMat(const boost::python::numpy::ndarray& ndarr);


#endif 