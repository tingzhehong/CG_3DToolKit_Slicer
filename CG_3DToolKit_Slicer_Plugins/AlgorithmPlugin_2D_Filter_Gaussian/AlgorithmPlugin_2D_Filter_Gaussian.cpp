#include "AlgorithmPlugin_2D_Filter_Gaussian.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_2D_Filter_Gaussian::AlgorithmPlugin_2D_Filter_Gaussian(AlgorithmInterface *parent) : AlgorithmInterface(parent)
{
    m_Type = ALG2D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::yellow;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::yellow;

    m_NodeBlock.Name = tr(u8"高斯滤波");
    m_NodeBlock.Title = tr(u8"高斯滤波");
    m_NodeBlock.Input << in_0;
    m_NodeBlock.Output << out_0;

    _theSizeArgs.ARG = tr(u8"滤波核尺寸");
    _theSizeArgs.VALUE = 3;
}

CG_NODEBLOCK *AlgorithmPlugin_2D_Filter_Gaussian::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_2D_Filter_Gaussian::AlgorithmPluginName()
{
    QString PluginName = tr(u8"高斯滤波");
    return PluginName;
}

QString AlgorithmPlugin_2D_Filter_Gaussian::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_2D_Filter_Gaussian::AlgorithmPluginID()
{
    return 0xca;
}

void AlgorithmPlugin_2D_Filter_Gaussian::SetAlgorithmInputData(QVector<QVariant> &datas)
{
    int num = datas.size();
    if (num != 1) {
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
}

QVector<QVariant> AlgorithmPlugin_2D_Filter_Gaussian::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_imgDst));
    datas << outdata;

    return datas;
}

void AlgorithmPlugin_2D_Filter_Gaussian::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 1) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _theSizeArgs = args.at(0);
    _theSize = (int)_theSizeArgs.VALUE;
    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_2D_Filter_Gaussian::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _theSizeArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_2D_Filter_Gaussian::GetAlgorithmShowData()
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

void AlgorithmPlugin_2D_Filter_Gaussian::Compute()
{
    alg::GaussianFilter(_imgSrc, _imgDst, _theSize);

    _computed = true;

    emit SignalComputed();
}

AlgorithmInterface *AlgorithmPlugin_2D_Filter_Gaussian::Clone()
{
    AlgorithmPlugin_2D_Filter_Gaussian *p = new AlgorithmPlugin_2D_Filter_Gaussian();
    return dynamic_cast<AlgorithmInterface *>(p);
}
