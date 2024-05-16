#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;

// TODO:

namespace alg
{

void ICP(PointCloudT::Ptr cloud_source, PointCloudT::Ptr cloud_target, PointCloudT::Ptr cloud_result, Eigen::Matrix4f &M_icp, const int iterations)
{
    std::cout << "ICP registration..." << endl;
    std::cout << "Loaded Size: " << cloud_source->size() << " data points from source" << std::endl;
    std::cout << "Loaded Size: " << cloud_target->size() << " data points from target" << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    icp.setMaximumIterations(iterations);
    icp.align(*cloud_result);

    M_icp = icp.getFinalTransformation();

    std::cout << M_icp << std::endl;
}

void NDT(PointCloudT::Ptr cloud_source, PointCloudT::Ptr cloud_target, PointCloudT::Ptr cloud_result, Eigen::Matrix4f &M_ndt, const int iterations)
{
    std::cout << "NDT registration..." << endl;
    std::cout << "Loaded Size: " << cloud_source->size() << " data points from source" << std::endl;
    std::cout << "Loaded Size: " << cloud_target->size() << " data points from target" << std::endl;

    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setInputSource(cloud_source);
    ndt.setInputTarget(cloud_target);
    ndt.setMaximumIterations(iterations);
    ndt.align(*cloud_result);

    M_ndt = ndt.getFinalTransformation();

    std::cout << M_ndt << std::endl;
}

}
