#include "AlgorithmPlugin_3D_Filter_SOR.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_3D_Filter_SOR::AlgorithmPlugin_3D_Filter_SOR(AlgorithmInterface *parent) : AlgorithmInterface(parent)
{
    m_Type = ALG3D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::red;

    CG_PORT in_1;
    in_1.NUM = 1;
    in_1.CLR = Qt::cyan;
    in_1.VAL = 0.001f;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::red;

    m_NodeBlock.Name = tr(u8"SOR滤波");
    m_NodeBlock.Title = tr(u8"SOR滤波");
    m_NodeBlock.Input << in_0 << in_1;
    m_NodeBlock.Output << out_0;

    _thresArgs.ARG = tr(u8"阈值大小");
    _thresArgs.VALUE = 0.001f;

    _cloudSrc.reset(new PointCloudT);
    _cloudDst.reset(new PointCloudT);
}

CG_NODEBLOCK *AlgorithmPlugin_3D_Filter_SOR::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_3D_Filter_SOR::AlgorithmPluginName()
{
    QString PluginName = tr(u8"SOR滤波");
    return PluginName;
}

QString AlgorithmPlugin_3D_Filter_SOR::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_3D_Filter_SOR::AlgorithmPluginID()
{
    return 0x12e;
}

void AlgorithmPlugin_3D_Filter_SOR::SetAlgorithmInputData(QVector<QVariant> &datas)
{
    int num = datas.size();
    if (num != 2) {
        qDebug() << "Algorithm input data overflow.";
        emit SignalMessage("Algorithm input data overflow.");
        return;
    }

    if (datas.at(0).canConvert<PointCloudT::Ptr>())
        _cloudSrc = datas.at(0).value<PointCloudT::Ptr>();
    _thres = datas.at(1).toFloat();
    _thresArgs.VALUE = _thres;
}

QVector<QVariant> AlgorithmPlugin_3D_Filter_SOR::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_cloudDst));
    datas << outdata;

    return datas;
}

void AlgorithmPlugin_3D_Filter_SOR::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 1) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _thresArgs = args.at(0);
    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_3D_Filter_SOR::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _thresArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_3D_Filter_SOR::GetAlgorithmShowData()
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

void AlgorithmPlugin_3D_Filter_SOR::Compute()
{
    alg::SORFilter(_cloudSrc, _cloudDst, _thres);

    _computed = true;

    emit SignalComputed();
}

AlgorithmInterface *AlgorithmPlugin_3D_Filter_SOR::Clone()
{
    AlgorithmPlugin_3D_Filter_SOR *p = new AlgorithmPlugin_3D_Filter_SOR();
    return dynamic_cast<AlgorithmInterface *>(p);
}
