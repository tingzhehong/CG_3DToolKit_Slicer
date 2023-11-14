#include "AlgorithmPlugin_2D_Filter_ReduceDomain.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_2D_Filter_ReduceDomain::AlgorithmPlugin_2D_Filter_ReduceDomain(AlgorithmInterface *parent) : AlgorithmInterface(parent)
{
    m_Type = ALG2D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::yellow;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::yellow;

    m_NodeBlock.Name = tr(u8"图像ROI");
    m_NodeBlock.Title = tr(u8"图像ROI");
    m_NodeBlock.Input << in_0;
    m_NodeBlock.Output << out_0;

    _ROI_X_Args.ARG = tr(u8"X");
    _ROI_Y_Args.ARG = tr(u8"Y");
    _ROI_W_Args.ARG = tr(u8"Width");
    _ROI_H_Args.ARG = tr(u8"Height");
    _ROI_X_Args.VALUE = 0;
    _ROI_Y_Args.VALUE = 0;
    _ROI_W_Args.VALUE = 100;
    _ROI_H_Args.VALUE = 100;
}

CG_NODEBLOCK *AlgorithmPlugin_2D_Filter_ReduceDomain::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_2D_Filter_ReduceDomain::AlgorithmPluginName()
{
    QString PluginName = tr(u8"图像ROI");
    return PluginName;
}

QString AlgorithmPlugin_2D_Filter_ReduceDomain::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_2D_Filter_ReduceDomain::AlgorithmPluginID()
{
    return 0xcc;
}

void AlgorithmPlugin_2D_Filter_ReduceDomain::SetAlgorithmInputData(QVector<QVariant> &datas)
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

QVector<QVariant> AlgorithmPlugin_2D_Filter_ReduceDomain::GetAlgorithmOutputData()
{
    QVector<QVariant> datas;
    QVariant srcImg, dstImg;
    dstImg.setValue(QVariant::fromValue(_imgDst));
    srcImg.setValue(QVariant::fromValue(_imgSrc));
    datas << dstImg;

    return datas;
}

void AlgorithmPlugin_2D_Filter_ReduceDomain::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 4) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _ROI_X_Args = args.at(0);
    _ROI_Y_Args = args.at(1);
    _ROI_W_Args = args.at(2);
    _ROI_H_Args = args.at(3);
    _ROI.x = _ROI_X_Args.VALUE;
    _ROI.y = _ROI_Y_Args.VALUE;
    _ROI.width = _ROI_W_Args.VALUE;
    _ROI.height = _ROI_H_Args.VALUE;
    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_2D_Filter_ReduceDomain::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _ROI_X_Args << _ROI_Y_Args << _ROI_W_Args << _ROI_H_Args;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_2D_Filter_ReduceDomain::GetAlgorithmShowData()
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

void AlgorithmPlugin_2D_Filter_ReduceDomain::Compute()
{
    cv::Rect R(0, 0, _imgSrc.cols, _imgSrc.rows);
    if (R.contains(cv::Point2f(_ROI.x, _ROI.y)) && R.contains(cv::Point2f(_ROI.x + _ROI.width, _ROI.y + _ROI.height)))
        alg::ReduceDomain(_imgSrc, _imgDst, _ROI);
    else
        emit SignalMessage("ROI's rectangle out of image range!");

    _computed = true;

    emit SignalComputed();
}

AlgorithmInterface *AlgorithmPlugin_2D_Filter_ReduceDomain::Clone()
{
    AlgorithmPlugin_2D_Filter_ReduceDomain *p = new AlgorithmPlugin_2D_Filter_ReduceDomain();
    return dynamic_cast<AlgorithmInterface *>(p);
}
