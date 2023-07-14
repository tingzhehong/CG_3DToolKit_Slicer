#ifndef NODEBLOCKWIDGET_H
#define NODEBLOCKWIDGET_H

#include <QWidget>
#include <AlgorithmInterface.h>

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

private:
    void InitUi();
    void InitConnections();
    void InitTableWidget();
    void ClearPointCloud();

protected:
    void PointCloud2VTKActor(PointCloudT::Ptr cloud, vtkActor *actor);

private:
    QTableWidget *m_ArgumentsTable;
    CGVTKWidget *m_CGVTKWidget;

    cv::Mat _image;
    PointCloudT::Ptr _cloud;

private:
    static NodeBlockWidget *m_NodeBlockWidget;
};

#endif // NODEBLOCKWIDGET_H
