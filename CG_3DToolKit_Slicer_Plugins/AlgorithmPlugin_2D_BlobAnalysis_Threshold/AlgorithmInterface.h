#ifndef ALGORITHMINTERFACE_H
#define ALGORITHMINTERFACE_H

#pragma once

#include <QObject>
#include <QColor>
#include <QVariant>
#include <QMap>
#include <iostream>
#include <fstream>
#include <sstream>

// PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

// OpenCV
#include <opencv2/opencv.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


enum CG_ALGORITHM_TYPE
{
    ALG2D,
    ALG3D,
};

struct CG_IMG
{
    cv::Mat DepthImage;
    cv::Mat ColorImage;
    cv::Mat GrayImage;
    cv::Mat IntensityImage;
};

struct CG_PORT
{
    int NUM;
    QColor CLR;
    QVariant VAL;
};

struct CG_ARGUMENT
{
    QString ARG;
    float VALUE;
};

struct CG_SHOWDATA
{
    CG_ALGORITHM_TYPE Type;
    QVariant Data;
};

struct CG_NODEBLOCK
{
    QString Name;
    QString Title;
    QList<CG_PORT> Input;
    QList<CG_PORT> Output;
};

class AlgorithmInterface : public QObject
{
    Q_OBJECT

public:
    explicit AlgorithmInterface(QObject *parent = nullptr);
    ~AlgorithmInterface() = default;

signals:
    void SignalMessage(const QString msg);
    void SignalComputed();

public:
    virtual CG_NODEBLOCK *CreatAlgorithmPlugin() = 0;
    virtual QString AlgorithmPluginName() = 0;
    virtual QString AlogorithmPlugVersion() = 0;
    virtual int AlgorithmPluginID() = 0;

    virtual void SetAlgorithmInputData(QVector<QVariant> &datas) = 0;
    virtual QVector<QVariant> GetAlgorithmOutputData() = 0;

    virtual void SetAlgorithmArguments(QVector<CG_ARGUMENT> &args) = 0;
    virtual QVector<CG_ARGUMENT> GetAlgorithmArguments() = 0;

    virtual CG_SHOWDATA GetAlgorithmShowData() = 0;

    virtual void Compute() = 0;

public:
    CG_ALGORITHM_TYPE m_Type;
};

#define AlgorithmInterface_iid "com.Interface.AlgorithmInterface"

Q_DECLARE_INTERFACE(AlgorithmInterface, AlgorithmInterface_iid)
Q_DECLARE_METATYPE(cv::Mat)
Q_DECLARE_METATYPE(CG_IMG)
Q_DECLARE_METATYPE(PointT)
Q_DECLARE_METATYPE(PointCloudT::Ptr)

#endif // ALGORITHMINTERFACE_H
