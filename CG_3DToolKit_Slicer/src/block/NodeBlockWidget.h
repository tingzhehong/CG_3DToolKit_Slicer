#ifndef NODEBLOCKWIDGET_H
#define NODEBLOCKWIDGET_H

#include <QWidget>
#include <AlgorithmInterface.h>
#include <CGPropertiesRegulator.h>

class QLabel;
class QTableWidget;
class QTableWidgetItem;
class CGVTKWidget;
class vtkActor;
class NodeBlockWidget : public QWidget
{
    Q_OBJECT

private:
    explicit NodeBlockWidget(QWidget *parent = nullptr);

public:
    static NodeBlockWidget *getInstance();

    void LoadAlgorithmArguments(QVector<CG_ARGUMENT> &args);
    void LoadAlgorithmShowData(CG_SHOWDATA &data);
    void SetCurrentAlgorithmPlugin(AlgorithmInterface *plugin);

private:
    void InitUi();
    void InitConnections();
    void InitTableWidget();
    void ClearPointCloud();

protected:
    void PointCloud2VTKActor(PointCloudT::Ptr cloud, vtkActor *actor);

public slots:
    void OnTableWidgetItemChanged(QTableWidgetItem *current);
    void OnSendAlgorithmArguuments();

private:
    QTableWidget *m_ArgumentsTable;
    CGVTKWidget *m_CGVTKWidget;

    cv::Mat _image;
    PointCloudT::Ptr _cloud;

    AlgorithmInterface *m_plugin;
    QVector<CG_ARGUMENT> m_args;

private:
    static NodeBlockWidget *m_NodeBlockWidget;
};

#endif // NODEBLOCKWIDGET_H
