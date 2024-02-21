#include "AlgorithmPlugin_3D_Match_Registration.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_3D_Match_Registration::AlgorithmPlugin_3D_Match_Registration(AlgorithmInterface *parent) : AlgorithmInterface(parent)
{
    m_Type = ALG3D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::red;

    CG_PORT in_1;
    in_1.NUM = 1;
    in_1.CLR = Qt::red;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::red;

    CG_PORT out_1;
    out_1.NUM = 1;
    out_1.CLR = Qt::green;

    m_NodeBlock.Name = tr(u8"点云配准");
    m_NodeBlock.Title = tr(u8"点云配准");
    m_NodeBlock.Input << in_0 << in_1;
    m_NodeBlock.Output << out_0 << out_1;

    _cloudSource.reset(new PointCloudT);
    _cloudTarget.reset(new PointCloudT);
    _cloudResult.reset(new PointCloudT);
}

CG_NODEBLOCK *AlgorithmPlugin_3D_Match_Registration::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_3D_Match_Registration::AlgorithmPluginName()
{
    QString PluginName = tr(u8"点云配准");
    return PluginName;
}

QString AlgorithmPlugin_3D_Match_Registration::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_3D_Match_Registration::AlgorithmPluginID()
{
    return 0x134;
}

void AlgorithmPlugin_3D_Match_Registration::SetAlgorithmInputData(QVector<QVariant> &datas)
{
    int num = datas.size();
    if (num != 2) {
        qDebug() << "Algorithm input data overflow.";
        emit SignalMessage("Algorithm input data overflow.");
        return;
    }

    if (datas.at(0).canConvert<PointCloudT::Ptr>())
        _cloudSource = datas.at(0).value<PointCloudT::Ptr>();
    if (datas.at(1).canConvert<PointCloudT::Ptr>())
        _cloudTarget = datas.at(1).value<PointCloudT::Ptr>();
}

QVector<QVariant> AlgorithmPlugin_3D_Match_Registration::GetAlgorithmOutputData()
{
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_cloudResult));

    QVariant outjson;
    outjson.setValue(QVariant::fromValue(_outJson));

    QVector<QVariant> datas;
    datas << outdata << outjson;

    return datas;
}

void AlgorithmPlugin_3D_Match_Registration::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 0) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_3D_Match_Registration::GetAlgorithmArguments()
{
    CG_ARGUMENT arg;
    arg.ARG = "Maximum Iterations";
    arg.VALUE= 100;

    QVector<CG_ARGUMENT> args;
    args << arg;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_3D_Match_Registration::GetAlgorithmShowData()
{
    CG_SHOWDATA showdata;

        if (_computed) {
            showdata.Type = ALG3D;
            showdata.Data.setValue(QVariant::fromValue(_cloudSource));
        }
        else {
            showdata.Type = ALG3D;
            showdata.Data.setValue(QVariant::fromValue(_cloudResult));
        }

        return showdata;
}

void AlgorithmPlugin_3D_Match_Registration::Compute()
{
    Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
    alg::ICP(_cloudSource, _cloudTarget, _cloudResult, M);

    QJsonObject json;
    json["Matrix 0"] = QString::number(M(0, 0), 'f', 6) + ", " + QString::number(M(0, 1), 'f', 6) + ", " + QString::number(M(0, 2), 'f', 6) + ", " + QString::number(M(0, 3), 'f', 6);
    json["Matrix 1"] = QString::number(M(1, 0), 'f', 6) + ", " + QString::number(M(1, 1), 'f', 6) + ", " + QString::number(M(1, 2), 'f', 6) + ", " + QString::number(M(1, 3), 'f', 6);
    json["Matrix 2"] = QString::number(M(2, 0), 'f', 6) + ", " + QString::number(M(2, 1), 'f', 6) + ", " + QString::number(M(2, 2), 'f', 6) + ", " + QString::number(M(2, 3), 'f', 6);
    json["Matrix 3"] = QString::number(M(3, 0), 'f', 6) + ", " + QString::number(M(3, 1), 'f', 6) + ", " + QString::number(M(3, 2), 'f', 6) + ", " + QString::number(M(3, 3), 'f', 6);
    _outJson = json;

    _computed = true;

    emit SignalComputed();
}

AlgorithmInterface *AlgorithmPlugin_3D_Match_Registration::Clone()
{
    AlgorithmPlugin_3D_Match_Registration *p = new AlgorithmPlugin_3D_Match_Registration();
    return dynamic_cast<AlgorithmInterface *>(p);
}
