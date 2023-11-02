#ifndef ALGORITHMPLUGIN_2D_Filter_Gaussian_H
#define ALGORITHMPLUGIN_2D_Filter_Gaussian_H

#include <QtPlugin>
#include <AlgorithmInterface.h>
#include <AlgorithmFuction.h>

class AlgorithmPlugin_2D_Filter_Gaussian : public AlgorithmInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "AlgorithmPlugin_2D_Filter_Gaussian")
    Q_INTERFACES(AlgorithmInterface)

public:
    explicit AlgorithmPlugin_2D_Filter_Gaussian(AlgorithmInterface *parent = nullptr);
    ~AlgorithmPlugin_2D_Filter_Gaussian() = default;

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
    int _theSize;
    CG_ARGUMENT _theSizeArgs;
    bool _computed;
};

#endif // ALGORITHMPLUGIN_2D_Filter_Gaussian_H
