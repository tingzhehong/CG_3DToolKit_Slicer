//CGImageFormatConvert.h
//

#ifndef CGIMAGEFORMATCONVER_H
#define CGIMAGEFORMATCONVER_H

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <QImage>

namespace CG
{

// cv::Mat转QImage
QImage CVMat2QImage(const cv::Mat &mat);
// QImage转cv::Mat
cv::Mat QImage2CVMat(const QImage &image, bool inCloneImageData);

}

#endif // CGIMAGEFORMATCONVER_H
