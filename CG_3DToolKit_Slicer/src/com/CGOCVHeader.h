#ifndef CGOCVHEADER_H
#define CGOCVHEADER_H

#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

struct CG_IMG
{
    cv::Mat DepthImage;
    cv::Mat ColorImage;
    cv::Mat GrayImage;
    cv::Mat IntensityImage;
};

extern CG_IMG g_Image;

#endif // CGOCVHEADER_H
