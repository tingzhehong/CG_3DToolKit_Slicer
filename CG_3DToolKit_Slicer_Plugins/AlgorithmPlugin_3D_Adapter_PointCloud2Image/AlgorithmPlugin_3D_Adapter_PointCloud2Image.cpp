#include "AlgorithmPlugin_3D_Adapter_PointCloud2Image.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_3D_Adapter_PointCloud2Image::AlgorithmPlugin_3D_Adapter_PointCloud2Image(AlgorithmInterface *parent) : AlgorithmInterface(parent)
{
    m_Type = ALG3D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::red;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::yellow;

    m_NodeBlock.Name = tr(u8"点云转图像");
    m_NodeBlock.Title = tr(u8"点云转图像");
    m_NodeBlock.Input << in_0;
    m_NodeBlock.Output << out_0;

    _XPitchArgs.ARG = tr(u8"XPitch");
    _XPitchArgs.VALUE = 0.001f;
    _XPitch = 0.001f;
    _YPitchArgs.ARG = tr(u8"YPitch");
    _YPitchArgs.VALUE = 0.001f;
    _YPitch = 0.001f;

    _colNumArgs.ARG = tr(u8"图像宽度");
    _colNumArgs.VALUE = 1024;
    _colNum = 1024;
    _rowNumArgs.ARG = tr(u8"图像高度");
    _rowNumArgs.VALUE = 1024;
    _rowNum = 1024;

    _cloudSrc.reset(new PointCloudT);
}

CG_NODEBLOCK *AlgorithmPlugin_3D_Adapter_PointCloud2Image::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_3D_Adapter_PointCloud2Image::AlgorithmPluginName()
{
    QString PluginName = tr(u8"点云转图像");
    return PluginName;
}

QString AlgorithmPlugin_3D_Adapter_PointCloud2Image::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_3D_Adapter_PointCloud2Image::AlgorithmPluginID()
{
    return 0x12c;
}

void AlgorithmPlugin_3D_Adapter_PointCloud2Image::SetAlgorithmInputData(QVector<QVariant> &datas)
{
    int num = datas.size();
    if (num != 1) {
        qDebug() << "Algorithm input data overflow.";
        emit SignalMessage("Algorithm input data overflow.");
        return;
    }

    if (datas.at(0).canConvert<PointCloudT::Ptr>())
       _cloudSrc = datas.at(0).value<PointCloudT::Ptr>();
}

QVector<QVariant> AlgorithmPlugin_3D_Adapter_PointCloud2Image::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_IMG));
    datas << outdata;

    return datas;
}

void AlgorithmPlugin_3D_Adapter_PointCloud2Image::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 4) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _XPitchArgs = args.at(0);
    _YPitchArgs = args.at(1);
    _colNumArgs = args.at(2);
    _rowNumArgs = args.at(3);

    _XPitch = _XPitchArgs.VALUE;
    _YPitch = _YPitchArgs.VALUE;
    _colNum = _colNumArgs.VALUE;
    _rowNum = _rowNumArgs.VALUE;

    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_3D_Adapter_PointCloud2Image::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _XPitchArgs << _YPitchArgs << _colNumArgs << _rowNumArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_3D_Adapter_PointCloud2Image::GetAlgorithmShowData()
{
    CG_SHOWDATA showdata;

    if (_computed) {
        showdata.Type = ALG2D;
        showdata.Data.setValue(QVariant::fromValue(_IMG.ColorImage));
    }
    else {
        showdata.Type = ALG3D;
        showdata.Data.setValue(QVariant::fromValue(_cloudSrc));
    }

    return showdata;
}

AlgorithmInterface *AlgorithmPlugin_3D_Adapter_PointCloud2Image::Clone()
{
    AlgorithmPlugin_3D_Adapter_PointCloud2Image *p = new AlgorithmPlugin_3D_Adapter_PointCloud2Image();
    return dynamic_cast<AlgorithmInterface *>(p);
}

void AlgorithmPlugin_3D_Adapter_PointCloud2Image::Compute()
{
    alg::CreateImageALL(_cloudSrc, _IMG.DepthImage, _IMG.GrayImage, _IMG.IntensityImage, _XPitch, _YPitch, _rowNum, _colNum);
    alg::DepthMat2ColorMat(_IMG.DepthImage, _IMG.ColorImage);

    _computed = true;

    emit SignalComputed();
}


