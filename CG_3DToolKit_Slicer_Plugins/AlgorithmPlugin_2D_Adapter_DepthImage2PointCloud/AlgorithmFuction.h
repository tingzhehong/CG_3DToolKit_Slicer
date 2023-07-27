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
    void FromDepthImage2PointCloud(cv::Mat &ImageDepth, float XPitch, float YPitch, float DownLimitThres, float UpLimitThres, PointCloudT::Ptr cloud);
}

#endif // ALGORITHMFUCTION_H
