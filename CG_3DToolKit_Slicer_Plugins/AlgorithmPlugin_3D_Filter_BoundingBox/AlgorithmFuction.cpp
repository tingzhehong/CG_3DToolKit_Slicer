#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::PassThroughFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const string Direction, const float LimitsDown, const float LimitsUp, const bool Negative)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(Direction);
    pass.setFilterLimits(LimitsDown, LimitsUp);
    pass.setFilterLimitsNegative(Negative);
    pass.filter(*filter_cloud);
}

