#include "AlgorithmPlugin_2D_Match_MatchTemplate.h"
#include <QList>
#include <QVector>
#include <QDebug>

AlgorithmPlugin_2D_Match_MatchTemplate::AlgorithmPlugin_2D_Match_MatchTemplate(AlgorithmInterface *parent) : AlgorithmInterface(parent)
{
    m_Type = ALG2D;

    CG_PORT in_0;
    in_0.NUM = 0;
    in_0.CLR = Qt::yellow;

    CG_PORT in_1;
    in_1.NUM = 1;
    in_1.CLR = Qt::yellow;

    CG_PORT out_0;
    out_0.NUM = 0;
    out_0.CLR = Qt::yellow;

    CG_PORT out_1;
    out_1.NUM = 1;
    out_1.CLR = Qt::green;

    m_NodeBlock.Name = tr(u8"模板匹配");
    m_NodeBlock.Title = tr(u8"模板匹配");
    m_NodeBlock.Input << in_0 << in_1;
    m_NodeBlock.Output << out_0 << out_1;

    _theNumberArgs.ARG = tr(u8"匹配数量");
    _theNumberArgs.VALUE = 1;

    _theMaxScoreArgs.ARG = tr(u8"匹配分数(max)");
    _theMaxScoreArgs.VALUE = 0;

    _theMinScoreArgs.ARG = tr(u8"匹配分数(min)");
    _theMinScoreArgs.VALUE = 0;
}

CG_NODEBLOCK *AlgorithmPlugin_2D_Match_MatchTemplate::CreatAlgorithmPlugin()
{
    return &m_NodeBlock;
}

QString AlgorithmPlugin_2D_Match_MatchTemplate::AlgorithmPluginName()
{
    QString PluginName = tr(u8"模板匹配");
    return PluginName;
}

QString AlgorithmPlugin_2D_Match_MatchTemplate::AlogorithmPlugVersion()
{
    QString PluginVersion = "ver 1.0.0";
    return PluginVersion;
}

int AlgorithmPlugin_2D_Match_MatchTemplate::AlgorithmPluginID()
{
    return 0xcd;
}

void AlgorithmPlugin_2D_Match_MatchTemplate::SetAlgorithmInputData(QVector<QVariant> &datas)
{
    int num = datas.size();
    if (num != 2) {
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

    _imgDst = _imgSrc.clone();

    if (datas.at(1).canConvert<CG_IMG>()) {
        _IMG = datas.at(1).value<CG_IMG>();
        _imgTemplate  = _IMG.GrayImage;
     }
    else if (datas.at(1).canConvert<cv::Mat>()) {
        _imgTemplate = datas.at(1).value<cv::Mat>();
    }
}

QVector<QVariant> AlgorithmPlugin_2D_Match_MatchTemplate::GetAlgorithmOutputData()
{ 
    QVariant outdata;
    outdata.setValue(QVariant::fromValue(_imgDst));

    QVariant outjson;
    outjson.setValue(QVariant::fromValue(_outJson));

    QVector<QVariant> datas;
    datas << outdata << outjson;

    return datas;
}

void AlgorithmPlugin_2D_Match_MatchTemplate::SetAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    int num = args.size();
    if (num != 3) {
        qDebug() << "Algorithm arguments overflow.";
        emit SignalMessage("Algorithm arguments overflow.");
        return;
    }

    _theNumberArgs = args.at(0);
    _theNumber = (int)_theNumberArgs.VALUE;
    _theMaxScoreArgs.VALUE = (float)_theMaxVal;
    _theMinScoreArgs.VALUE = (float)_theMinVal;

    _computed = false;
}

QVector<CG_ARGUMENT> AlgorithmPlugin_2D_Match_MatchTemplate::GetAlgorithmArguments()
{
    QVector<CG_ARGUMENT> args;
    args << _theNumberArgs << _theMaxScoreArgs << _theMinScoreArgs;

    return args;
}

CG_SHOWDATA AlgorithmPlugin_2D_Match_MatchTemplate::GetAlgorithmShowData()
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

void AlgorithmPlugin_2D_Match_MatchTemplate::Compute()
{
    std::vector<cv::Point> vec;
    vec = alg::MatchTemplate(_imgSrc, _imgTemplate, _theMaxVal, _theMinVal, _theNumber);

    for (auto it = _outJson.begin(); it != _outJson.end(); ++it)
    {
        _outJson.erase(it);
    }

    QJsonObject json;
    int i = 1;
    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
        std::cout << (*it).x << "  " << (*it).y;
        cv::rectangle(_imgDst, cv::Point((*it).x, (*it).y), cv::Point((*it).x + _imgTemplate.cols, (*it).y + _imgTemplate.rows), cv::Scalar(0, 255, 0), 3, 3, 0);

        json["pos " + QString::number(i) + " x"] = (*it).x;
        json["pos " + QString::number(i) + " y"] = (*it).y;
        ++i;
    }
    _outJson = json;
    _theMaxScoreArgs.VALUE = (float)_theMaxVal;
    _theMinScoreArgs.VALUE = (float)_theMinVal;

    _computed = true;

    emit SignalComputed();
}

AlgorithmInterface *AlgorithmPlugin_2D_Match_MatchTemplate::Clone()
{
    AlgorithmPlugin_2D_Match_MatchTemplate *p = new AlgorithmPlugin_2D_Match_MatchTemplate();
    return dynamic_cast<AlgorithmInterface *>(p);
}
