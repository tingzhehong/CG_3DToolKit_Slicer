#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::SectionPlaneProfile(PointCloudT::Ptr src_cloud, PointCloudT::Ptr dst_cloud, const float thres, const CG_Plane plane)
{
    dst_cloud->clear();

    size_t num = src_cloud->size();
    if (num == 0) return;

    for (size_t i = 0; i < num; ++i)
    {
        float diff = ConformPlaneFourmula(src_cloud->points[i], plane);

        if (fabs(diff) <= thres)
        {
            PointT pt = src_cloud->points[i];
            dst_cloud->push_back(pt);
        }
    }
}

