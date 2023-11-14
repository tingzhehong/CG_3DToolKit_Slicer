#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::ReduceDomain(cv::Mat &imgSrc, cv::Mat &imgDst, cv::Rect2f &roi)
{
    cv::Mat dst(imgSrc, roi);
    imgDst = dst.clone();
}

