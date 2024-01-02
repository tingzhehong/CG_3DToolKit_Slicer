#ifndef ALGORITHMFUCTION_H
#define ALGORITHMFUCTION_H

#include <AlgorithmInterface.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>
#include <string>

// TODO:

struct CG_Plane
{
    float A;
    float B;
    float C;
    float D;

    CG_Plane()
    {
        A = 0;
        B = 0;
        C = 0;
        D = 0;
    }
};

namespace alg
{
    /**
    * @brief        直通滤波 PassThroughFilter
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/
    void PassThroughFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const std::string Direction, const float LimitsDown, const float LimitsUp, const bool Negative);

    /**
    * @brief        拟合平面 FittingPlane
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/
    void FittingPlane(PointCloudT::Ptr cloud, CG_Plane &plane, const float thres);

    /**
    * @brief        倾斜校正 PlaneCorrection
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/
    Eigen::Matrix4f PlaneCorrection(CG_Plane &plane, CG_Plane &normal);

    /**
    * @brief        作用于点云 AffinePointCloud
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/
    void AffinePointCloud(PointCloudT::Ptr cloud, const Eigen::Matrix4f rotateMatrix);
}

#endif // ALGORITHMFUCTION_H
