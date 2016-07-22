#pragma once
#include <opencv2/opencv.hpp>
#include "detector.hpp"


class HOGDetector : public Detector {
public:
	HOGDetector(cv::Size winSize = cv::Size(80,40), cv::Size blockSize = cv::Size(16,16), cv::Size blockStride = cv::Size(8,8), cv::Size cellSize = cv::Size(8,8), int nbins = 9):hog(winSize,blockSize,blockStride,cellSize,nbins){};
	virtual~HOGDetector(){};

	virtual void load(const char* model);
	virtual std::vector<cv::Rect> detect(const cv::Mat& image);
	
protected:
	cv::HOGDescriptor hog;
};




