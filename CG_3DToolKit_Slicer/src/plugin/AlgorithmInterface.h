#ifndef ALGORITHMINTERFACE_H
#define ALGORITHMINTERFACE_H

#include <QObject>
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


class AlgorithmInterface : public QObject
{
    Q_OBJECT

public:
    explicit AlgorithmInterface(QObject *parent = nullptr);
    ~AlgorithmInterface() = default;

signals:
    void SignalMessage(const QString msg);
    void SignalExcuted();

public:
    virtual QObject *CreatAlgorithmPlugin() = 0;
    virtual QString AlgorithmPluginName() = 0;
    virtual QString AlogorithmPlugVersion() = 0;
    virtual int AlgorithmPluginID() = 0;


    virtual void Compute() = 0;
};

#define AlgorithmInterface_iid "com.Interface.AlgorithmInterface"

Q_DECLARE_INTERFACE(AlgorithmInterface, AlgorithmInterface_iid)
Q_DECLARE_METATYPE(cv::Mat)
Q_DECLARE_METATYPE(PointT)
Q_DECLARE_METATYPE(PointCloudT::Ptr)

#endif // ALGORITHMINTERFACE_H
