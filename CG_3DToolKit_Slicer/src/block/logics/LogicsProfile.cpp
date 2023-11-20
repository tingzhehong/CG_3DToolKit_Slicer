#include "LogicsProfile.h"
#include "CGMetaType.h"
#include <QDebug>

static void DepthMat2ColorMat(cv::Mat &depthImage, cv::Mat &colorImage)
{
    double min_z = 0, max_z = 0;
    cv::minMaxIdx(depthImage, &min_z, &max_z);
    float ramp = (max_z - min_z) / 50;

    int indexcolor = 0;
    cv::Mat img(depthImage.rows, depthImage.cols, CV_8UC3, cv::Scalar(0));

    for (int i = 0; i < depthImage.rows; ++i)
    {
        for (int j = 0; j < depthImage.cols; ++j)
        {
            indexcolor = ceil((depthImage.at<float>(i, j) - min_z) / ramp);
            if (indexcolor < 0) { indexcolor = 0; }
            if (indexcolor > 50) { indexcolor = 50; }

            img.at<cv::Vec3b>(i, j)[0] = colorLUT[50 - indexcolor].B * 255.0f;
            img.at<cv::Vec3b>(i, j)[1] = colorLUT[50 - indexcolor].G * 255.0f;
            img.at<cv::Vec3b>(i, j)[2] = colorLUT[50 - indexcolor].R * 255.0f;
        }
    }

    colorImage = img.clone();
}

static void DepthMat2GrayMat(cv::Mat &depthImage, cv::Mat &grayImage)
{
    double min_z = 0, max_z = 0;
    cv::minMaxIdx(depthImage, &min_z, &max_z);
    float ramp = (max_z - min_z) / 255;

    int indexcolor = 0;
    cv::Mat img(depthImage.rows, depthImage.cols, CV_8UC1, cv::Scalar(0));

    for (int i = 0; i < depthImage.rows; ++i)
    {
        for (int j = 0; j < depthImage.cols; ++j)
        {
            indexcolor = ceil((depthImage.at<float>(i, j) - min_z) / ramp);
            if (indexcolor < 0) { indexcolor = 0; }
            if (indexcolor > 255) { indexcolor = 255; }

            img.at<uchar>(i, j) = indexcolor;
        }
    }

    grayImage = img.clone();
}

LogicsProfile::LogicsProfile(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    NodeItemFactory(tr(u8"轮廓"), 2, 0);
    CreatNodeItem20(tr(u8"轮廓"));
}

void LogicsProfile::Run()
{
    QVariant var2d = m_NodeItem->portAt(1)->value();
    QVariant var3d = m_NodeItem->portAt(0)->value();

    CG_IMG IMG;
    if (var2d.canConvert<CG_IMG>()) {
        IMG = var2d.value<CG_IMG>();
        g_Image.ColorImage = IMG.ColorImage.clone();
        g_Image.DepthImage = IMG.DepthImage.clone();
        g_Image.GrayImage = IMG.GrayImage.clone();
        g_Image.IntensityImage = IMG.IntensityImage.clone();

        emit SignalShow2D();
    }

    if (var3d.canConvert<PointCloudT::Ptr>()) {
        g_PointCloud = var3d.value<PointCloudT::Ptr>();

        emit SignalShow3D();
    }

    m_IsRuned = true;
}

NodeItem *LogicsProfile::CreatNodeItem20(const QString nodename)
{
    m_NodeItem->setTitle(nodename);
    m_NodeItem->setNodeName(nodename);
    m_NodeItem->portAt(1)->setColor(Qt::yellow);
    m_NodeItem->portAt(0)->setColor(Qt::red);

    return m_NodeItem;
}
