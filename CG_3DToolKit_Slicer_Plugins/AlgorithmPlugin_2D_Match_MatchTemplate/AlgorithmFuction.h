#ifndef ALGORITHMFUCTION_H
#define ALGORITHMFUCTION_H

#include <AlgorithmInterface.h>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonArray>

// TODO:

namespace alg
{
    /**
    * @brief        模板匹配 MatchTemplate
    * @param[in]
    * @param[out]
    * @return
    * @author
    **/
    std::vector<cv::Point> MatchTemplate(cv::Mat &imgSrc, cv::Mat &imgTemplate, double &theMaxVal, double &theMinVal, const int theNumber);

    cv::Point MatchNextMinLoc(cv::Mat &result, cv::Point &minLoc, double &maxValue, int templateW, int templateH);
    cv::Point MatchNextMaxLoc(cv::Mat &result, cv::Point &maxLoc, double &minValue, int templateW, int templateH);
}

#endif // ALGORITHMFUCTION_H
