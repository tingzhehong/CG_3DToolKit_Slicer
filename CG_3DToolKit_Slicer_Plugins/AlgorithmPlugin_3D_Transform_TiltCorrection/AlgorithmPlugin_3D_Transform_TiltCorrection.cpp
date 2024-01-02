#include "AlgorithmPlugin_3D_Transform_TiltCorrection.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_3D_Transform_TiltCorrection::AlgorithmPlugin_3D_Transform_TiltCorrection(AlgorithmInterface *parent) : AlgorithmInterface(parent)
{
    m_Type = ALG3D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::red;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::red;

    m_NodeBlock.Name = tr(u8"倾斜校正");
    m_NodeBlock.Title = tr(u8"倾斜校正");
    m_NodeBlock.Input << in_0;
    m_NodeBlock.Output << out_0;

    _A_Args.ARG = tr(u8"A");
    _A_Args.VALUE = 0;
    _B_Args.ARG = tr(u8"B");
    _B_Args.VALUE = 0;
    _C_Args.ARG = tr(u8"C");
    _C_Args.VALUE = 1;

    _XminArgs.ARG = tr(u8"X方向最小 Xmin");
    _XminArgs.VALUE = -100.0f;
    _XmaxArgs.ARG = tr(u8"X方向最大 Xmax");
    _XmaxArgs.VALUE = 100.0f;

    _YminArgs.ARG = tr(u8"Y方向最小 Ymin");
    _YminArgs.VALUE = -100.0f;
    _YmaxArgs.ARG = tr(u8"Y方向最大 Ymax");
    _YmaxArgs.VALUE = 100.0f;

    _ZminArgs.ARG = tr(u8"Z方向最小 Zmin");
    _ZminArgs.VALUE = -100.0f;
    _ZmaxArgs.ARG = tr(u8"Z方向最大 Zmax");
    _ZmaxArgs.VALUE = 100.0f;

    _cloudSrc.reset(new PointCloudT);
    _cloudDst.reset(new PointCloudT);
    _cloudPlane.reset(new PointCloudT);
}

CG_NODEBLOCK *AlgorithmPlugin_3D_Transform_TiltCorrection::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_3D_Transform_TiltCorrection::AlgorithmPluginName()
{
    QString PluginName = tr(u8"倾斜校正");
    return PluginName;
}

QString AlgorithmPlugin_3D_Transform_TiltCorrection::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_3D_Transform_TiltCorrection::AlgorithmPluginID()
{
    return 0x132;
}

void AlgorithmPlugin_3D_Transform_TiltCorrection::SetAlgorithmInputData(QVector<QVariant> &datas)
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

QVector<QVariant> AlgorithmPlugin_3D_Transform_TiltCorrection::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_cloudDst));
    datas << outdata;

    return datas;
}

void AlgorithmPlugin_3D_Transform_TiltCorrection::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 9) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _A_Args = args.at(0);
    _B_Args = args.at(1);
    _C_Args = args.at(2);

    _A = _A_Args.VALUE;
    _B = _B_Args.VALUE;
    _C = _C_Args.VALUE;

    _XminArgs = args.at(3);
    _XmaxArgs = args.at(4);
    _YminArgs = args.at(5);
    _YmaxArgs = args.at(6);
    _ZminArgs = args.at(7);
    _ZmaxArgs = args.at(8);

    _Xmin = _XminArgs.VALUE;
    _Xmax = _XmaxArgs.VALUE;
    _Ymin = _YminArgs.VALUE;
    _Ymax = _YmaxArgs.VALUE;
    _Zmin = _ZminArgs.VALUE;
    _Zmax = _ZmaxArgs.VALUE;

    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_3D_Transform_TiltCorrection::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _A_Args << _B_Args << _C_Args
         << _XminArgs << _XmaxArgs
         << _YminArgs << _YmaxArgs
         << _ZminArgs << _ZmaxArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_3D_Transform_TiltCorrection::GetAlgorithmShowData()
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

void AlgorithmPlugin_3D_Transform_TiltCorrection::Compute()
{
    bool _negative = false;
    alg::PassThroughFilter(_cloudSrc,   _cloudPlane, "x", _Xmin, _Xmax, _negative);
    alg::PassThroughFilter(_cloudPlane, _cloudPlane, "y", _Ymin, _Ymax, _negative);
    alg::PassThroughFilter(_cloudPlane, _cloudPlane, "z", _Zmin, _Zmax, _negative);

    CG_Plane _plane;
    alg::FittingPlane(_cloudPlane, _plane, 0.005);

    CG_Plane _normal;
    _normal.A = _A, _normal.B = _B, _normal.C = _C;
    Eigen::Matrix4f RT = Eigen::Matrix4f::Identity();
    RT = alg::PlaneCorrection(_plane, _normal);

    pcl::copyPointCloud(*_cloudSrc, *_cloudDst);
    alg::AffinePointCloud(_cloudDst, RT);

    _computed = true;

    emit SignalComputed();
}

AlgorithmInterface *AlgorithmPlugin_3D_Transform_TiltCorrection::Clone()
{
    AlgorithmPlugin_3D_Transform_TiltCorrection *p = new AlgorithmPlugin_3D_Transform_TiltCorrection();
    return dynamic_cast<AlgorithmInterface *>(p);
}
