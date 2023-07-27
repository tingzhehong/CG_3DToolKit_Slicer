#ifndef ALGORITHMPLUGIN_3D_ADAPTER_POINTCLOUD2IMAGE_H
#define ALGORITHMPLUGIN_3D_ADAPTER_POINTCLOUD2IMAGE_H

#include <QtPlugin>
#include <AlgorithmInterface.h>
#include <AlgorithmFuction.h>

class AlgorithmPlugin_3D_Adapter_PointCloud2Image : public AlgorithmInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "AlgorithmPlugin_3D_Adapter_PointCloud2Image")
    Q_INTERFACES(AlgorithmInterface)

public:
    explicit AlgorithmPlugin_3D_Adapter_PointCloud2Image(AlgorithmInterface *parent = nullptr);
    ~AlgorithmPlugin_3D_Adapter_PointCloud2Image() = default;

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
    PointCloudT::Ptr _cloudSrc;

    float _XPitch;
    float _YPitch;
    float _colNum;
    float _rowNum;
    CG_ARGUMENT _XPitchArgs;
    CG_ARGUMENT _YPitchArgs;
    CG_ARGUMENT _colNumArgs;
    CG_ARGUMENT _rowNumArgs;

    bool _computed;
};

#endif // ALGORITHMPLUGIN_3D_ADAPTER_POINTCLOUD2IMAGE_H
