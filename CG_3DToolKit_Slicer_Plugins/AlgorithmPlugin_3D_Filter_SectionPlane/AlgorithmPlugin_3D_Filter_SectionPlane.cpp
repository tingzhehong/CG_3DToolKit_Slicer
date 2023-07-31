#include "AlgorithmPlugin_3D_Filter_SectionPlane.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_3D_Filter_SectionPlane::AlgorithmPlugin_3D_Filter_SectionPlane(AlgorithmInterface *parent) : AlgorithmInterface(parent)
  , _thres(0.01)
  , _A(0)
  , _B(0)
  , _C(0)
  , _X(0)
  , _Y(0)
  , _Z(0)
{
    m_Type = ALG3D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::red;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::red;

    m_NodeBlock.Name = tr(u8"平面裁切");
    m_NodeBlock.Title = tr(u8"平面裁切");
    m_NodeBlock.Input << in_0;
    m_NodeBlock.Output << out_0;

    _thresArgs.ARG = tr(u8"厚度阈值");
    _thresArgs.VALUE = 0.01f;

    _A_Args.ARG = tr(u8"A");
    _A_Args.VALUE = 0;
    _B_Args.ARG = tr(u8"B");
    _B_Args.VALUE = 0;
    _C_Args.ARG = tr(u8"C");
    _C_Args.VALUE = 1;

    _X_Args.ARG = tr(u8"X");
    _X_Args.VALUE = 0;
    _Y_Args.ARG = tr(u8"Y");
    _Y_Args.VALUE = 0;
    _Z_Args.ARG = tr(u8"Z");
    _Z_Args.VALUE = 0;

    _cloudSrc.reset(new PointCloudT);
    _cloudDst.reset(new PointCloudT);
}

CG_NODEBLOCK *AlgorithmPlugin_3D_Filter_SectionPlane::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_3D_Filter_SectionPlane::AlgorithmPluginName()
{
    QString PluginName = tr(u8"平面裁切");
    return PluginName;
}

QString AlgorithmPlugin_3D_Filter_SectionPlane::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_3D_Filter_SectionPlane::AlgorithmPluginID()
{
    return 0x130;
}

void AlgorithmPlugin_3D_Filter_SectionPlane::SetAlgorithmInputData(QVector<QVariant> &datas)
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

QVector<QVariant> AlgorithmPlugin_3D_Filter_SectionPlane::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_cloudDst));
    datas << outdata;

    return datas;
}

void AlgorithmPlugin_3D_Filter_SectionPlane::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 7) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _A_Args = args.at(0);
    _B_Args = args.at(1);
    _C_Args = args.at(2);

    _X_Args = args.at(3);
    _Y_Args = args.at(4);
    _Z_Args = args.at(5);

    _thresArgs = args.at(6);

    _A = _A_Args.VALUE;
    _B = _B_Args.VALUE;
    _C = _C_Args.VALUE;

    _X = _X_Args.VALUE;
    _Y = _Y_Args.VALUE;
    _Z = _Z_Args.VALUE;

    _thres = _thresArgs.VALUE;

    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_3D_Filter_SectionPlane::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _A_Args << _B_Args << _C_Args << _X_Args  << _Y_Args  << _Z_Args << _thresArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_3D_Filter_SectionPlane::GetAlgorithmShowData()
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

void AlgorithmPlugin_3D_Filter_SectionPlane::Compute()
{
    alg::CG_Plane Plane;
    Plane = alg::GetPlaneFormula(_A, _B, _C, _X, _Y, _Z);
    alg::SectionPlaneProfile(_cloudSrc, _cloudDst, _thres, Plane);

    _computed = true;

    emit SignalComputed();
}

AlgorithmInterface *AlgorithmPlugin_3D_Filter_SectionPlane::Clone()
{
    AlgorithmPlugin_3D_Filter_SectionPlane *p = new AlgorithmPlugin_3D_Filter_SectionPlane();
    return dynamic_cast<AlgorithmInterface *>(p);
}
