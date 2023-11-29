#ifndef ALGORITHMPLUGIN_2D_Match_MatchTemplate_H
#define ALGORITHMPLUGIN_2D_Match_MatchTemplate_H

#include <QtPlugin>
#include <AlgorithmInterface.h>
#include <AlgorithmFuction.h>

class AlgorithmPlugin_2D_Match_MatchTemplate : public AlgorithmInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "AlgorithmPlugin_2D_Match_MatchTemplate")
    Q_INTERFACES(AlgorithmInterface)

public:
    explicit AlgorithmPlugin_2D_Match_MatchTemplate(AlgorithmInterface *parent = nullptr);
    ~AlgorithmPlugin_2D_Match_MatchTemplate() = default;

public:
    CG_NODEBLOCK *CreatAlgorithmPlugin() override;
    QString AlgorithmPluginName() override;
    QString AlogorithmPlugVersion() override;
    int AlgorithmPluginID() override;

    void SetAlgorithmInputData(QVector<QVariant> &datas) override;
    QVector<QVariant> GetAlgorithmOutputData() override;

    void SetAlgorithmArguments(QVector<CG_ARGUMENT> &args) override;
    QVector<CG_ARGUMENT> GetAlgorithmArguments() override;

    CG_SHOWDATA GetAlgorithmShowData() override;
    AlgorithmInterface *Clone() override;

    void Compute() override;

protected:
    CG_NODEBLOCK m_NodeBlock;

private:
    CG_IMG _IMG;
    cv::Mat _imgSrc;
    cv::Mat _imgDst;
    cv::Mat _imgTemplate;
    double _theMinVal = 0;
    double _theMaxVal = 0;
    int _theNumber;
    CG_ARGUMENT _theNumberArgs;
    CG_ARGUMENT _theMinScoreArgs;
    CG_ARGUMENT _theMaxScoreArgs;
    QJsonObject _outJson;
    bool _computed;
};

#endif // ALGORITHMPLUGIN_2D_Match_MatchTemplate_H
