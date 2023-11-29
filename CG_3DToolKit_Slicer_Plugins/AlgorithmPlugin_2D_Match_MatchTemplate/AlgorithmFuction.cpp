#include <AlgorithmFuction.h>
#include <QDebug>

using namespace std;
using namespace cv;

// TODO:

std::vector<cv::Point> alg::MatchTemplate(cv::Mat &imgSrc, cv::Mat &imgTemplate, double &theMaxVal, double &theMinVal, const int theNumber)
{
    std::vector<cv::Point> MatchLoc;

    if (imgSrc.empty()) { cout << "原始图像为空！" << endl; return MatchLoc; }
    if (imgTemplate.empty()) { cout << "模板图像为空！" << endl; return MatchLoc; }
    if (imgTemplate.cols > imgSrc.cols || imgTemplate.rows > imgSrc.cols) { cout << "模板图像不能比原始图像大！" << endl; return MatchLoc; }

    int TemplateWidth = imgTemplate.cols;
    int TemplateHeight = imgTemplate.rows;

    cv::Mat result;

    int result_cols = imgSrc.cols - imgTemplate.cols + 1;
    int result_rows = imgSrc.rows - imgTemplate.rows + 1;
    result.create(result_cols, result_rows, CV_32FC1);

    cv::matchTemplate(imgSrc, imgTemplate, result, CV_TM_CCOEFF_NORMED);

    double minVal, maxVal;
    cv::Point minLoc, maxLoc;

    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());  MatchLoc.push_back(maxLoc);  theMaxVal = maxVal;

    for (int i = 1; i < theNumber; ++i)
    {
        cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
        std::cout << "maxVal =  " << maxVal << endl;
        theMinVal = maxVal;
        if (theMaxVal < maxVal)
            theMaxVal = maxVal;

        cv::Point new_maxLoc;
        new_maxLoc = MatchNextMaxLoc(result, maxLoc, minVal, TemplateWidth, TemplateHeight);
        MatchLoc.push_back(new_maxLoc);
    }

    return MatchLoc;
}


Point alg::MatchNextMinLoc(cv::Mat &result, cv::Point &minLoc, double &maxValue, int templateW, int templateH)
{
    int startX = minLoc.x - templateW / 3;
    int startY = minLoc.y - templateH / 3;
    int endX = minLoc.x + templateW / 3;
    int endY = minLoc.y + templateH / 3;

    if (startX < 0 || startY < 0)
    {
        startX = 0;
        startY = 0;
    }
    if (endX > result.cols - 1 || endY > result.rows - 1)
    {
        endX = result.cols - 1;
        endY = result.rows - 1;
    }

    int x, y;

    for (y = startY; y < endY; y++)
    {
        for (x = startX; x < endX; x++)
        {
            float *data = result.ptr<float>(y);

            data[x] = maxValue;
        }
    }

    double new_minValue, new_maxValue;
    Point new_minLoc, new_maxLoc;

    minMaxLoc(result, &new_minValue, &new_maxValue, &new_minLoc, &new_maxLoc, cv::Mat());

    return new_minLoc;
}

Point alg::MatchNextMaxLoc(cv::Mat &result, cv::Point &maxLoc, double &minValue, int templateW, int templateH)
{
    int startX = maxLoc.x - templateW / 3;
    int startY = maxLoc.y - templateH / 3;
    int endX = maxLoc.x + templateW / 3;
    int endY = maxLoc.y + templateH / 3;

    if (startX < 0 || startY < 0)
    {
        startX = 0;
        startY = 0;
    }
    if (endX > result.cols - 1 || endY > result.rows - 1)
    {
        endX = result.cols - 1;
        endY = result.rows - 1;
    }

    int x, y;

    for (y = startY; y < endY; y++)
    {
        for (x = startX; x < endX; x++)
        {
            float *data = result.ptr<float>(y);

            data[x] = minValue;
        }
    }

    double new_minValue, new_maxValue;
    Point new_minLoc, new_maxLoc;

    minMaxLoc(result, &new_minValue, &new_maxValue, &new_minLoc, &new_maxLoc, cv::Mat());

    return new_maxLoc;
}
