#ifndef ALGORITHMPLUGIN_3D_Filter_SectionPlane_H
#define ALGORITHMPLUGIN_3D_Filter_SectionPlane_H

#include <QtPlugin>
#include <AlgorithmInterface.h>
#include <AlgorithmFuction.h>

class AlgorithmPlugin_3D_Filter_SectionPlane : public AlgorithmInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "AlgorithmPlugin_3D_Filter_SectionPlane")
    Q_INTERFACES(AlgorithmInterface)

public:
    explicit AlgorithmPlugin_3D_Filter_SectionPlane(AlgorithmInterface *parent = nullptr);
    ~AlgorithmPlugin_3D_Filter_SectionPlane() = default;

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
    float _A, _B, _C;
    float _X, _Y, _Z;
    CG_ARGUMENT _thresArgs;
    CG_ARGUMENT _A_Args, _B_Args, _C_Args;
    CG_ARGUMENT _X_Args, _Y_Args, _Z_Args;
    
    bool _computed;
};

#endif // ALGORITHMPLUGIN_3D_Filter_SectionPlane_H
