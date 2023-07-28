#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::YYYFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const float thres)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> YYY;
    YYY.setInputCloud(cloud);
    YYY.setMeanK(6);
    YYY.setStddevMulThresh(thres);
    YYY.filter(*filter_cloud);
}

