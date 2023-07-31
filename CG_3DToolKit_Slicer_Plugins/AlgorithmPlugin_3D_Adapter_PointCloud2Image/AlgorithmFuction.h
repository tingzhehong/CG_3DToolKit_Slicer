#ifndef ALGORITHMFUCTION_H
#define ALGORITHMFUCTION_H

#include <AlgorithmInterface.h>

// TODO:

namespace alg
{
    /**
    * @brief        图像转点云 DepthImage2PointCloud
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/

    // 创建所有图像
    void CreateImageALL(PointCloudT::Ptr cloud, cv::Mat &imgDepth, cv::Mat &imgGray, cv::Mat &imgIntensity, const int &rowNum, const int &colNum);
    // 灰度图像至彩色图像
    void GrayMat2ColorMat(cv::Mat &grayImage, cv::Mat &colorImage);
    // 深度图像至彩色图像
    void DepthMat2ColorMat(cv::Mat &depthImage, cv::Mat &colorImage);

    struct CG_Color
    {
        float R;
        float G;
        float B;
    };

    static CG_Color colorLUT[51] = {
        //红
        1.0f, 0.0f, 0.0f,
        1.0f, 0.1f, 0.0f,
        1.0f, 0.2f, 0.0f,
        1.0f, 0.3f, 0.0f,
        1.0f, 0.4f, 0.0f,
        1.0f, 0.5f, 0.0f,
        1.0f, 0.6f, 0.0f,
        1.0f, 0.7f, 0.0f,
        1.0f, 0.8f, 0.0f,
        1.0f, 0.9f, 0.0f,
        //黄
        1.0f, 1.0f, 0.0f,
        0.9f, 1.0f, 0.0f,
        0.8f, 1.0f, 0.0f,
        0.7f, 1.0f, 0.0f,
        0.6f, 1.0f, 0.0f,
        0.5f, 1.0f, 0.0f,
        0.4f, 1.0f, 0.0f,
        0.3f, 1.0f, 0.0f,
        0.2f, 1.0f, 0.0f,
        0.1f, 1.0f, 0.0f,
        //绿
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.1f,
        0.0f, 1.0f, 0.2f,
        0.0f, 1.0f, 0.3f,
        0.0f, 1.0f, 0.4f,
        0.0f, 1.0f, 0.5f,
        0.0f, 1.0f, 0.6f,
        0.0f, 1.0f, 0.7f,
        0.0f, 1.0f, 0.8f,
        0.0f, 1.0f, 0.9f,
        //青
        0.0f, 1.0f, 1.0f,
        0.0f, 0.9f, 1.0f,
        0.0f, 0.8f, 1.0f,
        0.0f, 0.7f, 1.0f,
        0.0f, 0.6f, 1.0f,
        0.0f, 0.5f, 1.0f,
        0.0f, 0.4f, 1.0f,
        0.0f, 0.3f, 1.0f,
        0.0f, 0.2f, 1.0f,
        0.0f, 0.1f, 1.0f,
        //蓝
        0.0f, 0.0f, 1.0f,
        0.1f, 0.0f, 1.0f,
        0.2f, 0.0f, 1.0f,
        0.3f, 0.0f, 1.0f,
        0.4f, 0.0f, 1.0f,
        0.5f, 0.0f, 1.0f,
        0.6f, 0.0f, 1.0f,
        0.7f, 0.0f, 1.0f,
        0.8f, 0.0f, 1.0f,
        0.9f, 0.0f, 1.0f,
        //黑
        0.0f, 0.0f, 0.0f,
    };

}

#endif // ALGORITHMFUCTION_H
