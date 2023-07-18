#include "AlgorithmPlugin_2D_BlobAnalysis_Threshold.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_2D_BlobAnalysis_Threshold::AlgorithmPlugin_2D_BlobAnalysis_Threshold(AlgorithmInterface *parent) : AlgorithmInterface(parent)
{
    m_Type = ALG2D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::green;

    CG_PORT in_1;
    in_1.NUM = 1;
    in_1.CLR = Qt::cyan;
    in_1.VAL = 0.0f;

    CG_PORT in_2;
    in_2.NUM = 2;
    in_2.CLR = Qt::cyan;
    in_2.VAL = 0.0f;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::green;

    m_NodeBlock.Name = tr(u8"图像二值化");
    m_NodeBlock.Title = tr(u8"图像二值化");
    m_NodeBlock.Input << in_0 << in_1 << in_2;
    m_NodeBlock.Output << out_0;

    _thresUpArgs.ARG = tr(u8"阈值上限");
    _thresUpArgs.VALUE = 255;
    _thresDownArgs.ARG = tr(u8"阈值下限");
    _thresDownArgs.VALUE = 0;

}

CG_NODEBLOCK *AlgorithmPlugin_2D_BlobAnalysis_Threshold::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_2D_BlobAnalysis_Threshold::AlgorithmPluginName()
{
    QString PluginName = tr(u8"图像二值化");
    return PluginName;
}

QString AlgorithmPlugin_2D_BlobAnalysis_Threshold::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_2D_BlobAnalysis_Threshold::AlgorithmPluginID()
{
    return 0xc9;
}

void AlgorithmPlugin_2D_BlobAnalysis_Threshold::SetAlgorithmInputData(QVector<QVariant> &datas)
{
    int num = datas.size();
    if (num != 3) {
        qDebug() << "Algorithm input data overflow.";
        emit SignalMessage("Algorithm input data overflow.");
        return;
    }

    if (datas.at(0).canConvert<CG_IMG>())
       _IMG = datas.at(0).value<CG_IMG>();
    _imgSrc = _IMG.GrayImage;
    _thresUp = datas.at(1).toInt();
    _thresDown = datas.at(2).toInt();
    _thresUpArgs.VALUE = _thresUp;
    _thresDownArgs.VALUE = _thresDown;
}

QVector<QVariant> AlgorithmPlugin_2D_BlobAnalysis_Threshold::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_imgDst));
    datas << outdata;

    return datas;
}

void AlgorithmPlugin_2D_BlobAnalysis_Threshold::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 2) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _thresUpArgs = args.at(0);
    _thresDownArgs = args.at(1);
    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_2D_BlobAnalysis_Threshold::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _thresUpArgs <<_thresDownArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_2D_BlobAnalysis_Threshold::GetAlgorithmShowData()
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

void AlgorithmPlugin_2D_BlobAnalysis_Threshold::Compute()
{
    alg::Threshold(_imgSrc, _imgDst, _thresUp, _thresDown);

    _computed = true;

    emit SignalComputed();
}
