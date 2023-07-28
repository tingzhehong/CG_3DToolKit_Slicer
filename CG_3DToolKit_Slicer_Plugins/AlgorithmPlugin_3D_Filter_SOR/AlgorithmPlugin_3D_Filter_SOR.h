#ifndef ALGORITHMPLUGIN_3D_FILTER_SOR_H
#define ALGORITHMPLUGIN_3D_FILTER_SOR_H

#include <QtPlugin>
#include <AlgorithmInterface.h>
#include <AlgorithmFuction.h>

class AlgorithmPlugin_3D_Filter_SOR : public AlgorithmInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "AlgorithmPlugin_3D_Filter_SOR")
    Q_INTERFACES(AlgorithmInterface)

public:
    explicit AlgorithmPlugin_3D_Filter_SOR(AlgorithmInterface *parent = nullptr);
    ~AlgorithmPlugin_3D_Filter_SOR() = default;

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
    PointCloudT::Ptr _cloudSrc;
    PointCloudT::Ptr _cloudDst;
    float _thres;
    CG_ARGUMENT _thresArgs;
    bool _computed;
};

#endif // ALGORITHMPLUGIN_3D_FILTER_SOR_H
