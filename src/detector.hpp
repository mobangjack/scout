#pragma once

#include <opencv2/opencv.hpp>

class Detector
{
	public:
	Detector(){};
	virtual~Detector(){};
	
	virtual void load(const char* model) = 0;
	virtual std::vector<cv::Rect> detect(const cv::Mat& image) = 0;
	
};
