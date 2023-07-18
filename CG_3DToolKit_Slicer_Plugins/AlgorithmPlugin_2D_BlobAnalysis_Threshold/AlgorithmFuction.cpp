#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::Threshold(cv::Mat &imgSrc, cv::Mat &imgDst, const int thresUp, const int thresDown)
{
    cv::Mat BlobImage_1, BlobImage_2;
    cv::threshold(imgSrc, BlobImage_1, thresDown, 255, cv::THRESH_BINARY);
    cv::threshold(imgSrc, BlobImage_2, thresUp, 255, cv::THRESH_BINARY_INV);
    cv::bitwise_and(BlobImage_1, BlobImage_2, imgDst);
}
