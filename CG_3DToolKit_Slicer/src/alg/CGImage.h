//CGImage.h
//

#ifndef CGIMAGE_H
#define CGIMAGE_H

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <CGOCVHeader.h>
#include <CGPCLHeader.h>

using namespace std;
using namespace cv;

namespace CG
{

// 创建深度图像
void CreateImageDepth(PointCloudT::Ptr cloud, cv::Mat &img, const float &XPitch, const float &YPitch, const int &rowNum, const int &colNum);
// 创建灰度图像
void CreateImageGray(PointCloudT::Ptr cloud, cv::Mat &img, const float &XPitch, const float &YPitch, const int &rowNum, const int &colNum);
// 创建Intensity图像
void CreateImageIntensity(PointCloudT::Ptr cloud, cv::Mat &img, const float &XPitch, const float &YPitch, const int &rowNum, const int &colNum);
// 创建所有深度及Intensity图像
void CreateImageALL(PointCloudT::Ptr cloud, cv::Mat &imgDepth, cv::Mat &imgGray, cv::Mat &imgIntensity, const float &XPitch, const float &YPitch, const int &rowNum, const int &colNum);

// 图像存储
void ImageWrite(const string File, const cv::Mat &Image);

// 灰度图像至彩色图像
void GrayMat2ColorMat(cv::Mat &grayImage, cv::Mat &colorImage);
// 深度图像至彩色图像
void DepthMat2ColorMat(cv::Mat &depthImage, cv::Mat &colorImage);
// 深度图像至灰度图像
void DepthMat2GrayMat(cv::Mat &depthImage, cv::Mat &grayImage);

}

#endif // CGIMAGE_H
