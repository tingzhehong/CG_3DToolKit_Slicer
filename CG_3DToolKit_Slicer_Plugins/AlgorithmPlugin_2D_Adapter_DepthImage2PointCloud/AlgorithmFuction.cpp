#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::FromDepthImage2PointCloud(Mat &ImageDepth, float XPitch, float YPitch, float DownLimitThres, float UpLimitThres, PointCloudT::Ptr cloud)
{
    if (ImageDepth.empty()) return;

    int icol = ImageDepth.cols;
    int irow = ImageDepth.rows;

    cv::Mat imageDepth = ImageDepth;
    cv::Mat imageDepth_flip;
    cv::flip(imageDepth, imageDepth_flip, 0);

    pcl::PointXYZRGB TimPoint;

    for (int i = 0; i < imageDepth_flip.rows; i++)
    {
        for (int j = 0; j < imageDepth_flip.cols; j++)
        {
            float z = imageDepth_flip.at<float>(i, j);
            if (z <= DownLimitThres) continue;
            if (z >= UpLimitThres) continue;

            float r = 255;
            float g = 255;
            float b = 255;

            TimPoint.r = r;
            TimPoint.g = g;
            TimPoint.b = b;

            TimPoint.x = j * XPitch;
            TimPoint.y = i * YPitch;
            TimPoint.z = z;

            cloud->push_back(TimPoint);
        }
    }

    cloud->width = icol;
    cloud->height = irow;
    cloud->is_dense = false;
}
