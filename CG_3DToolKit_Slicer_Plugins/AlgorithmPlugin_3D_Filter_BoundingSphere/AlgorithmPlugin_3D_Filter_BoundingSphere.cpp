#include "AlgorithmPlugin_3D_Filter_BoundingSphere.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_3D_Filter_BoundingSphere::AlgorithmPlugin_3D_Filter_BoundingSphere(AlgorithmInterface *parent) : AlgorithmInterface(parent),
    _Side(0),
    _X(0), _Y(0), _Z(0),
    _Radius(1)
{
    m_Type = ALG3D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::red;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::red;

    _SideArgs.ARG = tr(u8"内侧/外侧 (0/1)");
    _SideArgs.VALUE = 0;

    _XArgs.ARG = tr(u8"球心 X");
    _XArgs.VALUE = _X;
    _YArgs.ARG = tr(u8"球心 Y");
    _YArgs.VALUE = _Y;
    _ZArgs.ARG = tr(u8"球心 Z");
    _ZArgs.VALUE = _Z;
    _RadiusArgs.ARG = tr(u8"球半径 R");
    _RadiusArgs.VALUE = _Radius;

    m_NodeBlock.Name = tr(u8"包围球裁切");
    m_NodeBlock.Title = tr(u8"包围球裁切");
    m_NodeBlock.Input << in_0;
    m_NodeBlock.Output << out_0;

    _cloudSrc.reset(new PointCloudT);
    _cloudDst.reset(new PointCloudT);
}

CG_NODEBLOCK *AlgorithmPlugin_3D_Filter_BoundingSphere::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_3D_Filter_BoundingSphere::AlgorithmPluginName()
{
    QString PluginName = tr(u8"包围球裁切");
    return PluginName;
}

QString AlgorithmPlugin_3D_Filter_BoundingSphere::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_3D_Filter_BoundingSphere::AlgorithmPluginID()
{
    return 0x133;
}

void AlgorithmPlugin_3D_Filter_BoundingSphere::SetAlgorithmInputData(QVector<QVariant> &datas)
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

QVector<QVariant> AlgorithmPlugin_3D_Filter_BoundingSphere::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_cloudDst));
    datas << outdata;

    return datas;
}

void AlgorithmPlugin_3D_Filter_BoundingSphere::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 5) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _SideArgs = args.at(0);
    _XArgs = args.at(1);
    _YArgs = args.at(2);
    _ZArgs = args.at(3);
    _RadiusArgs = args.at(4);

    _Side = _SideArgs.VALUE;
    _X = _XArgs.VALUE;
    _Y = _YArgs.VALUE;
    _Z = _ZArgs.VALUE;
    _Radius = _RadiusArgs.VALUE;

    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_3D_Filter_BoundingSphere::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _SideArgs << _XArgs << _YArgs << _ZArgs << _RadiusArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_3D_Filter_BoundingSphere::GetAlgorithmShowData()
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

void AlgorithmPlugin_3D_Filter_BoundingSphere::Compute()
{
    bool _Negative = false;

    if (_Side <= 0)
       _Negative = false;
    else
       _Negative = true;

    PointT _center;
    _center.x = _X;
    _center.y = _Y;
    _center.z = _Z;

    alg::RadiusFilter(_cloudSrc, _cloudDst, _center, _Radius, _Negative);

    _computed = true;

    emit SignalComputed();
}

AlgorithmInterface *AlgorithmPlugin_3D_Filter_BoundingSphere::Clone()
{
    AlgorithmPlugin_3D_Filter_BoundingSphere *p = new AlgorithmPlugin_3D_Filter_BoundingSphere();
    return dynamic_cast<AlgorithmInterface *>(p);
}
