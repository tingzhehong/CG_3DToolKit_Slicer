#include "AlgorithmPlugin_2D_BlobAnalysis_ImageRotate.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_2D_BlobAnalysis_ImageRotate::AlgorithmPlugin_2D_BlobAnalysis_ImageRotate(AlgorithmInterface *parent) : AlgorithmInterface(parent)
{
    m_Type = ALG2D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::yellow;

    CG_PORT in_1;
    in_1.NUM = 1;
    in_1.CLR = Qt::cyan;
    in_1.VAL = 1.0f;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::yellow;

    m_NodeBlock.Name = tr(u8"图像旋转");
    m_NodeBlock.Title = tr(u8"图像旋转");
    m_NodeBlock.Input << in_0 << in_1;
    m_NodeBlock.Output << out_0;

    _angleArgs.ARG = tr(u8"旋转角度");
    _angleArgs.VALUE = 1.0f;
}

CG_NODEBLOCK *AlgorithmPlugin_2D_BlobAnalysis_ImageRotate::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_2D_BlobAnalysis_ImageRotate::AlgorithmPluginName()
{
    QString PluginName = tr(u8"图像旋转");
    return PluginName;
}

QString AlgorithmPlugin_2D_BlobAnalysis_ImageRotate::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_2D_BlobAnalysis_ImageRotate::AlgorithmPluginID()
{
    return 0xce;
}

void AlgorithmPlugin_2D_BlobAnalysis_ImageRotate::SetAlgorithmInputData(QVector<QVariant> &datas)
{
    int num = datas.size();
    if (num != 2) {
        qDebug() << "Algorithm input data overflow.";
        emit SignalMessage("Algorithm input data overflow.");
        return;
    }

    if (datas.at(0).canConvert<CG_IMG>()) {
        _IMG = datas.at(0).value<CG_IMG>();
        _imgSrc = _IMG.GrayImage;
    }
    else if (datas.at(0).canConvert<cv::Mat>()) {
        _imgSrc = datas.at(0).value<cv::Mat>();
    }
    _angle = datas.at(1).toInt();
    _angleArgs.VALUE = _angle;
}

QVector<QVariant> AlgorithmPlugin_2D_BlobAnalysis_ImageRotate::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_imgDst));
    datas << outdata;

    return datas;
}

void AlgorithmPlugin_2D_BlobAnalysis_ImageRotate::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 1) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _angleArgs = args.at(0);
    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_2D_BlobAnalysis_ImageRotate::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _angleArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_2D_BlobAnalysis_ImageRotate::GetAlgorithmShowData()
{
    CG_SHOWDATA showdata;

    if (_computed) {
        showdata.Type = ALG2D;
        showdata.Data.setValue(QVariant::fromValue(_imgDst));
    }
    else {
        showdata.Type = ALG2D;
        showdata.Data.setValue(QVariant::fromValue(_imgSrc));
    }

    return showdata;
}

void AlgorithmPlugin_2D_BlobAnalysis_ImageRotate::Compute()
{
    _imgDst = _imgSrc.clone();

    alg::RotateImage(_imgDst, _angle);

    _computed = true;

    emit SignalComputed();
}

AlgorithmInterface *AlgorithmPlugin_2D_BlobAnalysis_ImageRotate::Clone()
{
    AlgorithmPlugin_2D_BlobAnalysis_ImageRotate *p = new AlgorithmPlugin_2D_BlobAnalysis_ImageRotate();
    return dynamic_cast<AlgorithmInterface *>(p);
}
