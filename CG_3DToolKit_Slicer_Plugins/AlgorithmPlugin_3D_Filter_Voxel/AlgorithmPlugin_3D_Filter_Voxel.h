#ifndef ALGORITHMPLUGIN_3D_FILTER_VOXEL_H
#define ALGORITHMPLUGIN_3D_FILTER_VOXEL_H

#include <QtPlugin>
#include <AlgorithmInterface.h>
#include <AlgorithmFuction.h>

class AlgorithmPlugin_3D_Filter_Voxel : public AlgorithmInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "AlgorithmPlugin_3D_Filter_Voxel")
    Q_INTERFACES(AlgorithmInterface)

public:
    explicit AlgorithmPlugin_3D_Filter_Voxel(AlgorithmInterface *parent = nullptr);
    ~AlgorithmPlugin_3D_Filter_Voxel() = default;

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

    void Compute() override;

protected:
    CG_NODEBLOCK m_NodeBlock;

private:
    PointCloudT::Ptr _cloudSrc;
    PointCloudT::Ptr _cloudDst;
    float _leaf;
    CG_ARGUMENT _leafArgs;
    bool _computed;
};

#endif // ALGORITHMPLUGIN_3D_FILTER_VOXEL_H
