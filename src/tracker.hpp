#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class Tracker
{
public:
	Tracker() {};
	 virtual ~Tracker() {};

     // Initialize tracker 
    virtual void init(const cv::Rect &roi, cv::Mat image) = 0;
    
    // Update position based on the new frame
    virtual cv::Rect update(cv::Mat image) = 0;
};

