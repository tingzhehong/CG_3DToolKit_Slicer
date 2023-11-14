#ifndef ALGORITHMPLUGIN_2D_Filter_ReduceDomain_H
#define ALGORITHMPLUGIN_2D_Filter_ReduceDomain_H

#include <QtPlugin>
#include <AlgorithmInterface.h>
#include <AlgorithmFuction.h>

class AlgorithmPlugin_2D_Filter_ReduceDomain : public AlgorithmInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "AlgorithmPlugin_2D_Filter_ReduceDomain")
    Q_INTERFACES(AlgorithmInterface)

public:
    explicit AlgorithmPlugin_2D_Filter_ReduceDomain(AlgorithmInterface *parent = nullptr);
    ~AlgorithmPlugin_2D_Filter_ReduceDomain() = default;

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
    cv::Rect2f _ROI;
    CG_ARGUMENT _ROI_X_Args;
    CG_ARGUMENT _ROI_Y_Args;
    CG_ARGUMENT _ROI_W_Args;
    CG_ARGUMENT _ROI_H_Args;
    bool _computed;
};

#endif // ALGORITHMPLUGIN_2D_Filter_ReduceDomain_H
