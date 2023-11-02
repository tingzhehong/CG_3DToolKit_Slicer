#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::GaussianFilter(cv::Mat &imgSrc, cv::Mat &imgDst, const int theSize)
{
    cv::GaussianBlur(imgSrc, imgDst, cv::Size(theSize, theSize), 0, 0);
}

