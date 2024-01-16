#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg:: RadiusFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const PointT center, const float radius, const bool Negative)
{
    PointCloudT::Ptr cc;
    cc.reset(new PointCloudT);
    pcl::copyPointCloud(*cloud, *cc);

    PointCloudT::Ptr pc;
    filter_cloud->clear();
    pc.reset(new PointCloudT);
    pc = filter_cloud;

    for (size_t i = 0; i < cc->size(); ++i)
    {
        PointT p;
        p.x = cc->points[i].x;
        p.y = cc->points[i].y;
        p.z = cc->points[i].z;
        p.r = cc->points[i].r;
        p.g = cc->points[i].g;
        p.b = cc->points[i].b;

        if (!Negative)
        {
            if (radius > PointDistance(p, center))
                pc->push_back(p);
        }
        else
        {
            if (radius < PointDistance(p, center))
                pc->push_back(p);
        }
    }
}

