#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::SORFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const float thres)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(6);
    sor.setStddevMulThresh(thres);
    sor.filter(*filter_cloud);
}

