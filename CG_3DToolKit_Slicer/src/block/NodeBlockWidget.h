﻿#ifndef NODEBLOCKWIDGET_H
#define NODEBLOCKWIDGET_H

#include <QWidget>
#include <AlgorithmInterface.h>
#include <CGPropertiesRegulator.h>
#include <CGVTKUtils.h>
#include <CGBoxWidgetObserver.h>
#include <CGSphereWidgetObserver.h>
#include <CGPlaneWidgetObserver.h>

class QLabel;
class QPushButton;
class QVBoxLayout;
class QTableWidget;
class QTableWidgetItem;
class QStackedWidget;
class QGraphicsPixmapItem;
class CGGraphicsScene;
class CGGraphicsView;
class CGShapeLineItem;
class CGShapeRectItem;
class CGShapeRotateRectangleItem;
class CGShapeCircleItem;
class CGShapeConcentricCircleItem;
class CGShapePolygonItem;
class CGVTKWidget;
class vtkActor;
class vtkBoxWidget;
class vtkSphereWidget;
class vtkPlaneWidget;
class NodeBlock;
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
    void SetCurrentNodeBlock(NodeBlock *block);
    void ShowAlgorithmPluginInfomation();
    void RemoveShapeItem();

private:
    void InitUi();
    void InitConnections();
    void InitTableWidget();
    void InitShapeItems();
    void InitBoxWidgetTools();
    void InitSphereWidgetTools();
    void InitPlaneWidgetTools();
    void ClearImage();
    void ClearPointCloud();

protected:
    void PointCloud2VTKActor(PointCloudT::Ptr cloud, vtkActor *actor);

signals:
    void SignalShapeItemValue(const QString msg);

public slots:
    void OnTableWidgetItemChanged(QTableWidgetItem *current);
    void OnSendAlgorithmArguments();
    void OnSetAlgorithmArguments(const QString value);

    QString ShapeItemValue();
    QString ToolBoxWidgetValue(vtkPlanes *planes);
    QString ToolSphereWidgetValue(double *sphere);
    QString ToolPlaneWidgetValue(vtkPlane* plane);

private:
    QTableWidget *m_ArgumentsTable;
    QStackedWidget *m_StackedWidget;
    CGGraphicsView *m_CGGraphicsView;
    CGVTKWidget *m_CGVTKWidget;
    CGVTKUtils::CGBoxWidgetObserver *m_CGBoxWidgeter;
    CGVTKUtils::CGSphereWidgetObserver *m_CGSphereWidgeter;
    CGVTKUtils::CGPlaneWidgetObserver *m_CGPlaneWidgeter;

    cv::Mat _image;
    PointCloudT::Ptr _cloud;
    vtkSmartPointer<vtkActor> _Actor;

    AlgorithmInterface *m_plugin;
    QVector<CG_ARGUMENT> m_args;
    NodeBlock *m_block;

private:
    QPixmap *pPixmap;
    QGraphicsPixmapItem *pItem;
    CGGraphicsScene *pScene;
    bool bGraphicsScene = false;

    QPushButton *p2DShapeLineBtn;
    QPushButton *p2DShapeRectBtn;
    QPushButton *p2DShapeRotatedRectBtn;
    QPushButton *p2DShapeCircleBtn;
    QPushButton *p2DShapeConcentricCircleBtn;
    QPushButton *p2DShapePolygonBtn;
    QPushButton *p2DShapeExecuteBtn;
    QPushButton *p2DShapeResetBtn;

    QPushButton *p3DShapeBoxBtn;
    QPushButton *p3DShapeSphereBtn;
    QPushButton *p3DShapePlaneBtn;
    QPushButton *p3DShapeExecuteBtn;
    QPushButton *p3DShapeResetBtn;

    QVBoxLayout *p2DShapeLayout;
    QVBoxLayout *p3DShapeLayout;

    QWidget *m_2DShapeWidget;
    QWidget *m_3DShapeWidget;

protected:
    enum ItemType
    {
        Line,                // 直线
        Rectangle,           // 矩形
        RotateRectangle,     // 旋转矩形
        Circle,              // 圆
        ConcentricCircle,    // 同心圆
        Polygon,             // 多边形
        BoundingBox,         // 包围盒
        BoundingSphere,      // 包围球
        Plane,               // 平面
    };
    ItemType m_CurrentShapeType;

    CGShapeLineItem *m_LineItem;
    CGShapeRectItem *m_RectItem;
    CGShapeRotateRectangleItem *m_RotateRectangleItem;
    CGShapeCircleItem *m_CircleItem;
    CGShapeConcentricCircleItem *m_ConcentricCircleItem;
    CGShapePolygonItem *m_PolygonItem;

    vtkSmartPointer<vtkBoxWidget> m_pBoxWidgetTool;
    vtkSmartPointer<vtkSphereWidget> m_pSphereWidgetTool;
    vtkSmartPointer<vtkPlaneWidget> m_pPlaneWidgetTool;

    bool IsShapeItem = false;
    QString StrToolBoxWidgetValue;
    QString StrToolSphereWidgetValue;
    QString StrToolPlaneWidgetValue;

private:
    static NodeBlockWidget *m_NodeBlockWidget;
};

#endif // NODEBLOCKWIDGET_H
