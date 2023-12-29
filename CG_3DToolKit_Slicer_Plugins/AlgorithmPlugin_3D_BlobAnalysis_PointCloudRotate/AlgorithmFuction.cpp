#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::RotatePointCloud(PointCloudT::Ptr cloud, int rotate, float angle)
{
	Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();

	switch (rotate) {
	case 0:
		rotationMatrix(1, 1) = cos(angle * M_PI / 180);
		rotationMatrix(1, 2) = -sin(angle * M_PI / 180);
		rotationMatrix(2, 1) = sin(angle * M_PI / 180);
		rotationMatrix(2, 2) = cos(angle * M_PI / 180);
		break;
	case 1:
		rotationMatrix(0, 0) = cos(angle * M_PI / 180);
		rotationMatrix(0, 2) = sin(angle * M_PI / 180);
		rotationMatrix(2, 0) = -sin(angle * M_PI / 180);
		rotationMatrix(2, 2) = cos(angle * M_PI / 180);
		break;
	case 2:
		rotationMatrix(0, 0) = cos(angle * M_PI / 180);
		rotationMatrix(0, 1) = -sin(angle * M_PI / 180);
		rotationMatrix(1, 0) = sin(angle * M_PI / 180);
		rotationMatrix(1, 1) = cos(angle * M_PI / 180);
		break;
	default:
		break;
	}

	pcl::transformPointCloud(*cloud, *cloud, rotationMatrix);
}
