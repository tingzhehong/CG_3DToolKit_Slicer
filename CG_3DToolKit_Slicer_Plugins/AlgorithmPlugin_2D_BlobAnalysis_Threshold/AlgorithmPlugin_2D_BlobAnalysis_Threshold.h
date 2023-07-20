#ifndef ALGORITHMPLUGIN_2D_BLOBANALYSIS_THRESHOLD_H
#define ALGORITHMPLUGIN_2D_BLOBANALYSIS_THRESHOLD_H

#include <QtPlugin>
#include <AlgorithmInterface.h>
#include <AlgorithmFuction.h>

class AlgorithmPlugin_2D_BlobAnalysis_Threshold : public AlgorithmInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "AlgorithmPlugin_2D_BlobAnalysis_Threshold")
    Q_INTERFACES(AlgorithmInterface)

public:
    explicit AlgorithmPlugin_2D_BlobAnalysis_Threshold(AlgorithmInterface *parent = nullptr);
    ~AlgorithmPlugin_2D_BlobAnalysis_Threshold() = default;

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
    int _thresUp;
    int _thresDown;
    CG_ARGUMENT _thresUpArgs;
    CG_ARGUMENT _thresDownArgs;
    bool _computed;
};

#endif // ALGORITHMPLUGIN_2D_BLOBANALYSIS_THRESHOLD_H
