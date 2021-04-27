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



#ifndef PARALLELFEATUREMATCHER_H_
#define PARALLELFEATUREMATCHER_H_

#include <opencv2/opencv.hpp>
#include <vector>

namespace mico {
	class ParallelFeatureMatcher : public cv::ParallelLoopBody {
	public:
		ParallelFeatureMatcher(const cv::Mat &_frame1, const cv::Mat &_frame2,
			const std::vector<cv::Point2i> &_kps, const std::vector<cv::Vec3f> &_epis,
			const std::pair<int, int> &_disparityRange,
			const int &_squareSize, const double &_maxTemplateScore,
			std::vector<std::vector<cv::Point2i>> &_points1, std::vector<std::vector<cv::Point2i>> &_points2,
			cv::Rect _vl, cv::Rect _vr);
		virtual void operator()(const cv::Range& range) const;

	private:
		cv::Point2i findMatch(const cv::Mat &_frame1, const cv::Mat &_frame2, const cv::Point2i &_point, const cv::Vec3f &_epiline, const std::pair<int, int> _disparityRange, const int _squareSize = 11) const;

	private:
		const cv::Mat &frame1, &frame2;
		const std::vector<cv::Point2i> &kps;
		const std::vector<cv::Vec3f> &epis;
		const std::pair<int, int> &disparityRange;
		const int &squareSize;
		const double &maxTemplateScore;
		std::vector<std::vector<cv::Point2i>> &points1;
		std::vector<std::vector<cv::Point2i>> &points2;
		cv::Rect validLeft;
		cv::Rect validRight;
	};

}	//	namespace mico 
#endif	//	PARALLELFEATUREMATCHER_H_
