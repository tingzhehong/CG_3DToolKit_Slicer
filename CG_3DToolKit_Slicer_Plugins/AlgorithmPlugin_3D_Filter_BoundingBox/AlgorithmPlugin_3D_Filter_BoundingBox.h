#ifndef ALGORITHMPLUGIN_3D_Filter_BoundingBox_H
#define ALGORITHMPLUGIN_3D_Filter_BoundingBox_H

#include <QtPlugin>
#include <AlgorithmInterface.h>
#include <AlgorithmFuction.h>

class AlgorithmPlugin_3D_Filter_BoundingBox : public AlgorithmInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "AlgorithmPlugin_3D_Filter_BoundingBox")
    Q_INTERFACES(AlgorithmInterface)

public:
    explicit AlgorithmPlugin_3D_Filter_BoundingBox(AlgorithmInterface *parent = nullptr);
    ~AlgorithmPlugin_3D_Filter_BoundingBox() = default;

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
    CG_ARGUMENT _SideArgs;
    float _Xmin, _Xmax;
    float _Ymin, _Ymax;
    float _Zmin, _Zmax;
    CG_ARGUMENT _XminArgs, _XmaxArgs;
    CG_ARGUMENT _YminArgs, _YmaxArgs;
    CG_ARGUMENT _ZminArgs, _ZmaxArgs;

    bool _computed;
};

#endif // ALGORITHMPLUGIN_3D_Filter_BoundingBox_H
