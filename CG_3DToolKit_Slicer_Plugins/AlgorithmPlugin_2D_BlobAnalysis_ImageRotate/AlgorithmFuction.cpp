#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

void alg::RotateImage(cv::Mat &image, const double angle)
{
    cv::Mat M;
    int w = image.cols;
    int h = image.rows;
    M = getRotationMatrix2D(Point2f(w / 2, h / 2), angle, 1);
    double cos = abs(M.at<double>(0, 0));
    double sin = abs(M.at<double>(0, 1));
    int nw = cos * w + sin * h;
    int nh = sin * w + cos * h;
    M.at<double>(0, 2) += (nw / 2 - w / 2);
    M.at<double>(1, 2) += (nh / 2 - h / 2);
    warpAffine(image, image, M, Size(nw, nh), INTER_LINEAR, 0);
}

