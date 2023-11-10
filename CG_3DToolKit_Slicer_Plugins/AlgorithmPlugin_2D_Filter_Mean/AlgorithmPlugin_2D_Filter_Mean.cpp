#include "AlgorithmPlugin_2D_Filter_Mean.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_2D_Filter_Mean::AlgorithmPlugin_2D_Filter_Mean(AlgorithmInterface *parent) : AlgorithmInterface(parent)
{
    m_Type = ALG2D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::yellow;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::yellow;

    m_NodeBlock.Name = tr(u8"均值滤波");
    m_NodeBlock.Title = tr(u8"均值滤波");
    m_NodeBlock.Input << in_0;
    m_NodeBlock.Output << out_0;

    _theSizeArgs.ARG = tr(u8"滤波核尺寸");
    _theSizeArgs.VALUE = 3;
}

CG_NODEBLOCK *AlgorithmPlugin_2D_Filter_Mean::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_2D_Filter_Mean::AlgorithmPluginName()
{
    QString PluginName = tr(u8"均值滤波");
    return PluginName;
}

QString AlgorithmPlugin_2D_Filter_Mean::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_2D_Filter_Mean::AlgorithmPluginID()
{
    return 0xcb;
}

void AlgorithmPlugin_2D_Filter_Mean::SetAlgorithmInputData(QVector<QVariant> &datas)
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

QVector<QVariant> AlgorithmPlugin_2D_Filter_Mean::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_imgDst));
    datas << outdata;

    return datas;
}

void AlgorithmPlugin_2D_Filter_Mean::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
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

QVector<CG_ARGUMENT> AlgorithmPlugin_2D_Filter_Mean::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _theSizeArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_2D_Filter_Mean::GetAlgorithmShowData()
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

void AlgorithmPlugin_2D_Filter_Mean::Compute()
{
    alg::MeanFilter(_imgSrc, _imgDst, _theSize);

    _computed = true;

    emit SignalComputed();
}

AlgorithmInterface *AlgorithmPlugin_2D_Filter_Mean::Clone()
{
    AlgorithmPlugin_2D_Filter_Mean *p = new AlgorithmPlugin_2D_Filter_Mean();
    return dynamic_cast<AlgorithmInterface *>(p);
}
