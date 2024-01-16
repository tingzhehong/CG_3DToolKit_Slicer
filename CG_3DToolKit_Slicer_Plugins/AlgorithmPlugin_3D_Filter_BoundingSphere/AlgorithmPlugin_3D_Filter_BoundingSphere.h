#ifndef ALGORITHMPLUGIN_3D_Filter_BoundingSphere_H
#define ALGORITHMPLUGIN_3D_Filter_BoundingSphere_H

#include <QtPlugin>
#include <AlgorithmInterface.h>
#include <AlgorithmFuction.h>

class AlgorithmPlugin_3D_Filter_BoundingSphere : public AlgorithmInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "AlgorithmPlugin_3D_Filter_BoundingSphere")
    Q_INTERFACES(AlgorithmInterface)

public:
    explicit AlgorithmPlugin_3D_Filter_BoundingSphere(AlgorithmInterface *parent = nullptr);
    ~AlgorithmPlugin_3D_Filter_BoundingSphere() = default;

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

    float _Side;
    float _X, _Y, _Z;
    float _Radius;

    CG_ARGUMENT _XArgs, _YArgs, _ZArgs;
    CG_ARGUMENT _RadiusArgs;
    CG_ARGUMENT _SideArgs;

    bool _computed;
};

#endif // ALGORITHMPLUGIN_3D_Filter_BoundingSphere_H
