#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::VoxelFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr sub_cloud, const float leaf)
{
    pcl::PCLPointCloud2::Ptr blob_cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr filtered_cloud(new pcl::PCLPointCloud2);

    pcl::toPCLPointCloud2(*cloud, *blob_cloud);
    pcl::VoxelGrid <pcl::PCLPointCloud2> VoxeGridlFilter;
    VoxeGridlFilter.setInputCloud(blob_cloud);
    VoxeGridlFilter.setLeafSize(leaf, leaf, leaf);
    VoxeGridlFilter.filter(*filtered_cloud);

    pcl::fromPCLPointCloud2(*filtered_cloud, *sub_cloud);
}

