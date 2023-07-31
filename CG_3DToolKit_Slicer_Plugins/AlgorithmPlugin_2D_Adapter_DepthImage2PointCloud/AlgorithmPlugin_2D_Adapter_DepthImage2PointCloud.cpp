#include "AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud::AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud(AlgorithmInterface *parent) : AlgorithmInterface(parent)
{
    m_Type = ALG2D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::yellow;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::red;

    m_NodeBlock.Name = tr(u8"图像转点云");
    m_NodeBlock.Title = tr(u8"图像转点云");
    m_NodeBlock.Input << in_0;
    m_NodeBlock.Output << out_0;

    _XPitchArgs.ARG = tr(u8"XPitch");
    _XPitchArgs.VALUE = 0.001f;
    _XPitch = 0.001f;
    _YPitchArgs.ARG = tr(u8"YPitch");
    _YPitchArgs.VALUE = 0.001f;
    _YPitch = 0.001f;

    _DownLimitThresArgs.ARG = tr(u8"下限值");
    _DownLimitThresArgs.VALUE = -10;
    _DownLimitThres = -10;
    _UpLimitThresArgs.ARG = tr(u8"上限值");
    _UpLimitThresArgs.VALUE = 99;
    _UpLimitThres = 99;

    _cloudDst.reset(new PointCloudT);
}

CG_NODEBLOCK *AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud::AlgorithmPluginName()
{
    QString PluginName = tr(u8"图像转点云");
    return PluginName;
}

QString AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud::AlgorithmPluginID()
{
    return 0xc8;
}

void AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud::SetAlgorithmInputData(QVector<QVariant> &datas)
{
    int num = datas.size();
    if (num != 1) {
        qDebug() << "Algorithm input data overflow.";
        emit SignalMessage("Algorithm input data overflow.");
        return;
    }

    if (datas.at(0).canConvert<CG_IMG>()) {
        _IMG = datas.at(0).value<CG_IMG>();
        _imgSrc = _IMG.DepthImage;
    }
    else if (datas.at(0).canConvert<cv::Mat>()) {
        _imgSrc = datas.at(0).value<cv::Mat>();
    }
}

QVector<QVariant> AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_cloudDst));
    datas << outdata;

    return datas;
}

void AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 4) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _XPitchArgs = args.at(0);
    _YPitchArgs = args.at(1);
    _UpLimitThresArgs = args.at(2);
    _DownLimitThresArgs = args.at(3);

    _XPitch = _XPitchArgs.VALUE;
    _YPitch = _YPitchArgs.VALUE;
    _UpLimitThres = _UpLimitThresArgs.VALUE;
    _DownLimitThres = _DownLimitThresArgs.VALUE;

    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _XPitchArgs << _YPitchArgs << _UpLimitThresArgs << _DownLimitThresArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud::GetAlgorithmShowData()
{
    CG_SHOWDATA showdata;
    showdata.Type = ALG3D;
    showdata.Data.setValue(QVariant::fromValue(_cloudDst));

    return showdata;
}

AlgorithmInterface *AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud::Clone()
{
    AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud *p = new AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud();
    return dynamic_cast<AlgorithmInterface *>(p);
}

void AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud::Compute()
{
    alg::FromDepthImage2PointCloud(_imgSrc, _XPitch, _YPitch, _DownLimitThres, _UpLimitThres, _cloudDst);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*_cloudDst, *_cloudDst, indices);

    _computed = true;

    emit SignalComputed();
}

