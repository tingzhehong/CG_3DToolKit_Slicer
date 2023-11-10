#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::MeanFilter(cv::Mat &imgSrc, cv::Mat &imgDst, const int theSize)
{
    cv::blur(imgSrc, imgDst, cv::Size(theSize, theSize));
}

