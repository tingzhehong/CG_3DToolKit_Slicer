#include "AlgorithmPlugin_3D_Filter_BoundingBox.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_3D_Filter_BoundingBox::AlgorithmPlugin_3D_Filter_BoundingBox(AlgorithmInterface *parent) : AlgorithmInterface(parent)
  , _Side(0)
  , _Xmin(-100)
  , _Xmax(100)
  , _Ymin(-100)
  , _Ymax(100)
  , _Zmin(-100)
  , _Zmax(100)
{
    m_Type = ALG3D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::red;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::red;

    m_NodeBlock.Name = tr(u8"包围盒裁切");
    m_NodeBlock.Title = tr(u8"包围盒裁切");
    m_NodeBlock.Input << in_0;
    m_NodeBlock.Output << out_0;

    _SideArgs.ARG = tr(u8"内侧/外侧 (0/1)");
    _SideArgs.VALUE = 0;

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
}

CG_NODEBLOCK *AlgorithmPlugin_3D_Filter_BoundingBox::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_3D_Filter_BoundingBox::AlgorithmPluginName()
{
    QString PluginName = tr(u8"包围盒裁切");
    return PluginName;
}

QString AlgorithmPlugin_3D_Filter_BoundingBox::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_3D_Filter_BoundingBox::AlgorithmPluginID()
{
    return 0x12f;
}

void AlgorithmPlugin_3D_Filter_BoundingBox::SetAlgorithmInputData(QVector<QVariant> &datas)
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

QVector<QVariant> AlgorithmPlugin_3D_Filter_BoundingBox::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_cloudDst));
    datas << outdata;

    return datas;
}

void AlgorithmPlugin_3D_Filter_BoundingBox::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 7) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _SideArgs = args.at(0);
    _XminArgs = args.at(1);
    _XmaxArgs = args.at(2);
    _YminArgs = args.at(3);
    _YmaxArgs = args.at(4);
    _ZminArgs = args.at(5);
    _ZmaxArgs = args.at(6);

    _Side = _SideArgs.VALUE;
    _Xmin = _XminArgs.VALUE;
    _Xmax = _XmaxArgs.VALUE;
    _Ymin = _YminArgs.VALUE;
    _Ymax = _YmaxArgs.VALUE;
    _Zmin = _ZminArgs.VALUE;
    _Zmax = _ZmaxArgs.VALUE;

    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_3D_Filter_BoundingBox::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _SideArgs
         << _XminArgs << _XmaxArgs
         << _YminArgs << _YmaxArgs
         << _ZminArgs << _ZmaxArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_3D_Filter_BoundingBox::GetAlgorithmShowData()
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

void AlgorithmPlugin_3D_Filter_BoundingBox::Compute()
{
    bool _Negative = false;

    if (_Side <= 0)
        _Negative = false;
    else
        _Negative = true;

    alg::PassThroughFilter(_cloudSrc, _cloudDst, "x", _Xmin, _Xmax, _Negative);
    alg::PassThroughFilter(_cloudDst, _cloudDst, "y", _Ymin, _Ymax, _Negative);
    alg::PassThroughFilter(_cloudDst, _cloudDst, "z", _Zmin, _Zmax, _Negative);

    _computed = true;

    emit SignalComputed();
}

AlgorithmInterface *AlgorithmPlugin_3D_Filter_BoundingBox::Clone()
{
    AlgorithmPlugin_3D_Filter_BoundingBox *p = new AlgorithmPlugin_3D_Filter_BoundingBox();
    return dynamic_cast<AlgorithmInterface *>(p);
}
