#ifndef ALGORITHMFUCTION_H
#define ALGORITHMFUCTION_H

#include <AlgorithmInterface.h>
#include <pcl/filters/radius_outlier_removal.h>

// TODO:

namespace alg
{
    /**
    * @brief        半径滤波 RadiusFilter
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/
    void RadiusFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const PointT center, const float radius, const bool Negative);

    inline float PointDistance(const PointT P1, const PointT P2)
    {
        float dist_square = (P1.x - P2.x) * (P1.x - P2.x) + (P1.y - P2.y) * (P1.y - P2.y) +  (P1.z - P2.z) * (P1.z - P2.z);
        float dist = sqrt(dist_square);
        return dist;
    }
}

#endif // ALGORITHMFUCTION_H
