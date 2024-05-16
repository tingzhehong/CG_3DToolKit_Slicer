#ifndef ALGORITHMFUCTION_H
#define ALGORITHMFUCTION_H

#include <AlgorithmInterface.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

// TODO:

namespace alg
{
    /**
    * @brief        Iterative Closest Point -- ICP
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/
    void ICP(PointCloudT::Ptr cloud_source, PointCloudT::Ptr cloud_target, PointCloudT::Ptr cloud_result, Eigen::Matrix4f &M_icp, const int iterations);

    /**
    * @brief        Normal Distributions Transform -- NDT
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/
    void NDT(PointCloudT::Ptr cloud_source, PointCloudT::Ptr cloud_target, PointCloudT::Ptr cloud_result, Eigen::Matrix4f &M_ndt, const int iterations);
}

#endif // ALGORITHMFUCTION_H
