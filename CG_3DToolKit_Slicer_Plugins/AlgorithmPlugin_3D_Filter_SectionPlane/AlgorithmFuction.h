#ifndef ALGORITHMFUCTION_H
#define ALGORITHMFUCTION_H

#include <AlgorithmInterface.h>

// TODO:

namespace alg
{
    struct CG_Plane
    {
        float A;
        float B;
        float C;
        float D;
    };

    inline CG_Plane GetPlaneFormula(const float A, const float B, const float C, const float X, const float Y, const float Z)
    {
        CG_Plane plane;

        plane.A = A;
        plane.B = B;
        plane.C = C;
        plane.D = (plane.A * X + plane.B * Y + plane.C * Z) * -1;

        return plane;
    }

    inline float ConformPlaneFourmula(const PointT pt, const CG_Plane plane)
    {
        float diff = plane.A * pt.x + plane.B * pt.y + plane.C * pt.z + plane.D;

        return diff;
    }

    /**
    * @brief        平面裁切 SectionPlaneProfile
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/
    void SectionPlaneProfile(PointCloudT::Ptr src_cloud, PointCloudT::Ptr dst_cloud, const float thres, const CG_Plane plane);

}

#endif // ALGORITHMFUCTION_H
