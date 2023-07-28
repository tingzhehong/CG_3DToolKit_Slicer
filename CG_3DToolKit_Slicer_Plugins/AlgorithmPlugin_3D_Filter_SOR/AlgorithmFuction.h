#ifndef ALGORITHMFUCTION_H
#define ALGORITHMFUCTION_H

#include <AlgorithmInterface.h>
#include <pcl/filters/statistical_outlier_removal.h>

// TODO:

namespace alg
{
    /**
    * @brief        SOR滤波 SORFilter
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/
    void SORFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const float thres);
}

#endif // ALGORITHMFUCTION_H
