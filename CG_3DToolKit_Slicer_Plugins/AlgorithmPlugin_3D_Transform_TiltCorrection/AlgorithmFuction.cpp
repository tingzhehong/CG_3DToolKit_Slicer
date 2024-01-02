#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::PassThroughFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const std::string Direction, const float LimitsDown, const float LimitsUp, const bool Negative)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(Direction);
    pass.setFilterLimits(LimitsDown, LimitsUp);
    pass.setFilterLimitsNegative(Negative);
    pass.filter(*filter_cloud);
}

void alg::FittingPlane(PointCloudT::Ptr cloud, CG_Plane &plane, const float thres)
{
    if (cloud->size() < 100) return;

    float A = 0;
    float B = 0;
    float C = 0;
    float D = 0;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // 拟合平面
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(thres);
    seg.setMaxIterations(100);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    A = coefficients->values[0];
    B = coefficients->values[1];
    C = coefficients->values[2];
    D = coefficients->values[3];
    //cout << "A = " << A << "  " << "B = " << B << "  " << "C = " << C << "  " << "D = " << D << endl;

    plane.A = A;
    plane.B = B;
    plane.C = C;
    plane.D = D;
}

Eigen::Matrix4f alg::PlaneCorrection(CG_Plane &plane, CG_Plane &normal)
{
    float A = plane.A;
    float B = plane.B;
    float C = plane.C;

    Eigen::Vector3d v1{ (double)A, (double)B, (double)C };
    Eigen::Vector3d v2{ (double)normal.A, (double)normal.B, (double)normal.C };

    if (C < 0) v2[2] = -1;

    Eigen::Matrix3d R;
    R = Eigen::Quaterniond::FromTwoVectors(v1, v2).toRotationMatrix();
    Eigen::Matrix3f rotationMatrix = R.cast<float>();

    Eigen::Vector3f transVector{ 0, 0, 1 };

    Eigen::Matrix4f RT = Eigen::Matrix4f::Identity();

    RT.block<3, 3>(0, 0) = rotationMatrix;
    RT.block<3, 1>(0, 3) = transVector;

    return RT;
}

void alg::AffinePointCloud(PointCloudT::Ptr cloud, const Eigen::Matrix4f rotateMatrix)
{
    pcl::transformPointCloud(*cloud, *cloud, rotateMatrix);
}

