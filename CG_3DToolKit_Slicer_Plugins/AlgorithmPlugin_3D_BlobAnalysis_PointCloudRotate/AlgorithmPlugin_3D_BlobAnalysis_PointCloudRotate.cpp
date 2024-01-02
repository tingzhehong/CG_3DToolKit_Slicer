#include "AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate::AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate(AlgorithmInterface *parent) : AlgorithmInterface(parent)
{
    m_Type = ALG3D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::red;

    CG_PORT in_1;
    in_1.NUM = 1;
    in_1.CLR = Qt::cyan;
    in_1.VAL = 1.0f;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::red;

    m_NodeBlock.Name = tr(u8"点云旋转");
    m_NodeBlock.Title = tr(u8"点云旋转");
    m_NodeBlock.Input << in_0 << in_1;
    m_NodeBlock.Output << out_0;

    _axisArgs.ARG = tr(u8"旋转轴向 (x:0 y:1 z:2)");
    _axisArgs.VALUE = 0;

    _angleArgs.ARG = tr(u8"旋转角度");
    _angleArgs.VALUE = 1.0f;

    _cloudSrc.reset(new PointCloudT);
    _cloudDst.reset(new PointCloudT);
}

CG_NODEBLOCK *AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate::AlgorithmPluginName()
{
    QString PluginName = tr(u8"点云旋转");
    return PluginName;
}

QString AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate::AlgorithmPluginID()
{
    return 0x131;
}

void AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate::SetAlgorithmInputData(QVector<QVariant> &datas)
{
    int num = datas.size();
    if (num != 2) {
        qDebug() << "Algorithm input data overflow.";
        emit SignalMessage("Algorithm input data overflow.");
        return;
    }

    if (datas.at(0).canConvert<PointCloudT::Ptr>())
        _cloudSrc = datas.at(0).value<PointCloudT::Ptr>();

    _axis = static_cast<int>(_axisArgs.VALUE);

    _angle = datas.at(1).toFloat();
    _angleArgs.VALUE = _angle;
}

QVector<QVariant> AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_cloudDst));
    datas << outdata;

    return datas;
}

void AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 2) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _axisArgs = args.at(0);
    _angleArgs = args.at(1);
    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _axisArgs << _angleArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate::GetAlgorithmShowData()
{
    CG_SHOWDATA showdata;

        if (_computed) {
            showdata.Type = ALG3D;
            showdata.Data.setValue(QVariant::fromValue(_cloudDst));
        }
        else {
            showdata.Type = ALG3D;
            showdata.Data.setValue(QVariant::fromValue(_cloudSrc));
        }

        return showdata;
}

void AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate::Compute()
{
    pcl::copyPointCloud(*_cloudSrc, *_cloudDst);

    alg::RotatePointCloud(_cloudDst, _axis, _angle);

    _computed = true;

    emit SignalComputed();
}

AlgorithmInterface *AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate::Clone()
{
    AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate *p = new AlgorithmPlugin_3D_BlobAnalysis_PointCloudRotate();
    return dynamic_cast<AlgorithmInterface *>(p);
}
