#ifndef ALGORITHMPLUGIN_3D_Match_Registration_H
#define ALGORITHMPLUGIN_3D_Match_Registration_H

#include <QtPlugin>
#include <AlgorithmInterface.h>
#include <AlgorithmFuction.h>

class AlgorithmPlugin_3D_Match_Registration : public AlgorithmInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "AlgorithmPlugin_3D_Match_Registration")
    Q_INTERFACES(AlgorithmInterface)

public:
    explicit AlgorithmPlugin_3D_Match_Registration(AlgorithmInterface *parent = nullptr);
    ~AlgorithmPlugin_3D_Match_Registration() = default;

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
    PointCloudT::Ptr _cloudSource;
    PointCloudT::Ptr _cloudTarget;
    PointCloudT::Ptr _cloudResult;
    QJsonObject _outJson;
    bool _computed;

    int _iterations;
    CG_ARGUMENT _iterationsArg;
};

#endif // ALGORITHMPLUGIN_3D_Match_Registration_H
