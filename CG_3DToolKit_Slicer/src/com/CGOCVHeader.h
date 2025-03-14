﻿#ifndef CGOCVHEADER_H
#define CGOCVHEADER_H

#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

enum CG_IMG_TYPE
{
    DEPTH,
    COLOR,
    GRAY,
    INTENSITY
};

struct CG_IMG
{
    cv::Mat DepthImage;
    cv::Mat ColorImage;
    cv::Mat GrayImage;
    cv::Mat IntensityImage;
};

extern CG_IMG g_Image;
extern float g_XPitch;
extern float g_YPitch;

#endif // CGOCVHEADER_H
