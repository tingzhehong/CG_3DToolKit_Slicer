#ifndef ALGORITHMFUCTION_H
#define ALGORITHMFUCTION_H

#include <AlgorithmInterface.h>
#include <pcl/filters/passthrough.h>
#include <string>

// TODO:

using std::string;

namespace alg
{
    /**
    * @brief        直通滤波 PassThroughFilter
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/
    void PassThroughFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const string Direction, const float LimitsDown, const float LimitsUp, const bool Negative);
}

#endif // ALGORITHMFUCTION_H
