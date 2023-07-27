#ifndef ALGORITHMPLUGIN_2D_ADAPTER_DEPTHIMAGE2POINTCLOUD_H
#define ALGORITHMPLUGIN_2D_ADAPTER_DEPTHIMAGE2POINTCLOUD_H

#include <QtPlugin>
#include <AlgorithmInterface.h>
#include <AlgorithmFuction.h>

class AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud : public AlgorithmInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud")
    Q_INTERFACES(AlgorithmInterface)

public:
    explicit AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud(AlgorithmInterface *parent = nullptr);
    ~AlgorithmPlugin_2D_Adapter_DepthImage2PointCloud() = default;

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
    PointCloudT::Ptr _cloudDst;

    float _XPitch;
    float _YPitch;
    float _DownLimitThres;
    float _UpLimitThres;
    CG_ARGUMENT _XPitchArgs;
    CG_ARGUMENT _YPitchArgs;
    CG_ARGUMENT _DownLimitThresArgs;
    CG_ARGUMENT _UpLimitThresArgs;

    bool _computed;
};

#endif // ALGORITHMPLUGIN_2D_ADAPTER_DEPTHIMAGE2POINTCLOUD_H
