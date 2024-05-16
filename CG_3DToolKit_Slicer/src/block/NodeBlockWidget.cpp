#include "NodeBlockWidget.h"
#include "NodeBlock.h"
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QHeaderView>
#include <QStackedWidget>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QDebug>
#include <CGGraphicsScene.h>
#include <QGraphicsItem>
#include <QGraphicsPixmapItem>
#include <CGGraphicsView.h>
#include <CGVTKHeader.h>
#include <CGVTKUtils.h>
#include <CGVTKWidget.h>
#include <CGImageFormatConvert.h>
#include <CGPropertiesForm2.h>
#include <vtkImageData.h>
#include <vtkImageActor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkBoxWidget.h>
#include <vtkSphereWidget.h>
#include <vtkPlaneWidget.h>
#include "CGShapeLineItem.h"
#include "CGShapeRectItem.h"
#include "CGShapeRotateRectangleItem.h"
#include "CGShapeCircleItem.h"
#include "CGShapeConcentricCircleItem.h"
#include "CGShapePolygonItem.h"


NodeBlockWidget *NodeBlockWidget::m_NodeBlockWidget = nullptr;

NodeBlockWidget::NodeBlockWidget(QWidget *parent) : QWidget(parent)
{
    InitUi();
    InitTableWidget();
    InitShapeItems();
    InitConnections();

    _cloud.reset(new PointCloudT);
}

NodeBlockWidget *NodeBlockWidget::getInstance()
{
    if (!m_NodeBlockWidget)
    {
        m_NodeBlockWidget = new NodeBlockWidget();
    }
    return m_NodeBlockWidget;
}

void NodeBlockWidget::LoadAlgorithmArguments(QVector<CG_ARGUMENT> &args)
{
    m_ArgumentsTable->clear();

    InitTableWidget();
    int num = args.size();
    if (num > 16)
    {
        m_ArgumentsTable->setRowCount(num);
    }

    for (int i = 0; i < num; ++i)
    {
        m_ArgumentsTable->setItem(i, 0, new QTableWidgetItem(QString::number(i + 1)));
        m_ArgumentsTable->setItem(i, 1, new QTableWidgetItem(args.at(i).ARG));
        m_ArgumentsTable->setItem(i, 2, new QTableWidgetItem(QString::number(args.at(i).VALUE, 'f', 6)));

        m_ArgumentsTable->item(i, 0)->setFlags(Qt::ItemIsEnabled);
        m_ArgumentsTable->item(i, 1)->setFlags(Qt::ItemIsEnabled);
    }
    m_ArgumentsTable->update();
}

void NodeBlockWidget::LoadAlgorithmShowData(CG_SHOWDATA &data)
{
    /*
    if (data.Type == CG_ALGORITHM_TYPE::ALG2D)
    {
        if (data.Data.canConvert<cv::Mat>())
            _image = data.Data.value<cv::Mat>();
        else
            return;

        QImage _Image = CG::CVMat2QImage(_image);

        ClearPointCloud();
        vtkSmartPointer<vtkImageData> _data = vtkSmartPointer<vtkImageData>::New();
        CGVTKUtils::qImageToVtkImage(_Image, _data);
        _Actor = vtkSmartPointer<vtkImageActor>::New();
        _Actor->SetInputData(_data);
        m_CGVTKWidget->addActor(_Actor, QColor(25, 50, 75));

        vtkSmartPointer<vtkInteractorStyleImage> style = vtkSmartPointer<vtkInteractorStyleImage>::New();
        style->SetDefaultRenderer(m_CGVTKWidget->defaultRenderer());

        m_CGVTKWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(style);
        m_CGVTKWidget->defaultRenderer()->GetActiveCamera()->SetPosition(0, 0, 1);
        m_CGVTKWidget->defaultRenderer()->GetActiveCamera()->SetFocalPoint(0, 0, 0);
        m_CGVTKWidget->defaultRenderer()->GetActiveCamera()->SetViewUp(0, 1, 0);
        m_CGVTKWidget->defaultRenderer()->ResetCamera();
        m_CGVTKWidget->update();
    }
    */

    if (data.Type == CG_ALGORITHM_TYPE::ALG2D)
    {
        if (data.Data.canConvert<cv::Mat>())
            _image = data.Data.value<cv::Mat>();
        else
            return;

        QImage _Image = CG::CVMat2QImage(_image);

        ClearImage();
        pPixmap->convertFromImage(_Image, Qt::AutoColor);
        pItem->setPixmap(*pPixmap);
        pScene->addItem(pItem);
        m_CGGraphicsView->setScene(pScene);
        m_CGGraphicsView->ImageWidth = pPixmap->width();
        m_CGGraphicsView->ImageHeight = pPixmap->height();
        m_CGGraphicsView->AutoFit();
        bGraphicsScene = true;

        m_StackedWidget->setCurrentWidget(m_CGGraphicsView);
        m_2DShapeWidget->setVisible(true);
        m_3DShapeWidget->setVisible(false);
    }

    if (data.Type == CG_ALGORITHM_TYPE::ALG3D)
    {
        if (data.Data.canConvert<PointCloudT::Ptr>())
            _cloud = data.Data.value<PointCloudT::Ptr>();
        else
            return;

        ClearPointCloud();
        _Actor = vtkSmartPointer<vtkActor>::New();
        PointCloud2VTKActor(_cloud, _Actor);
        m_CGVTKWidget->addActor(_Actor, QColor(25, 50, 75));

        vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
        style->SetDefaultRenderer(m_CGVTKWidget->defaultRenderer());

        m_CGVTKWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(style);
        m_CGVTKWidget->defaultRenderer()->ResetCamera();
        m_CGVTKWidget->update();

        m_StackedWidget->setCurrentWidget(m_CGVTKWidget);
        m_2DShapeWidget->setVisible(false);
        m_3DShapeWidget->setVisible(true);
    }
}

void NodeBlockWidget::SetCurrentAlgorithmPlugin(AlgorithmInterface *plugin)
{
    m_plugin = plugin;
    m_args = plugin->GetAlgorithmArguments();
}

void NodeBlockWidget::SetCurrentNodeBlock(NodeBlock *block)
{
    m_block = block;
}

void NodeBlockWidget::ShowAlgorithmPluginInfomation()
{
    CGPropertiesRegulator::getInstance()->m_Form2->m_PropertiesTree->clear();

    QTreeWidgetItem *pPluginItem = new QTreeWidgetItem(QStringList{tr(u8"算子属性")});

    QStringList propertiesList;
    QString name = m_plugin->AlgorithmPluginName();
    QString id = QString::number(m_plugin->AlgorithmPluginID());
    QString ver = m_plugin->AlogorithmPlugVersion();
    propertiesList.append(tr(u8"Node Block ID: ") + id);
    propertiesList.append(tr(u8"Node Block Name: ") + name);
    propertiesList.append(tr(u8"Node Block Version: ") + ver);

    for (int i = 0; i < propertiesList.size(); ++i) {
        QTreeWidgetItem *pItem = new QTreeWidgetItem(QStringList() << propertiesList.at(i));
        pPluginItem->addChild(pItem);
    }
    CGPropertiesRegulator::getInstance()->m_Form2->m_PropertiesTree->addTopLevelItem(pPluginItem);
    CGPropertiesRegulator::getInstance()->m_Form2->m_PropertiesTree->expandAll();
    CGPropertiesRegulator::getInstance()->m_Form2->update();
}

void NodeBlockWidget::InitUi()
{
    m_ArgumentsTable = new QTableWidget(this);
    m_ArgumentsTable->setShowGrid(true);
    m_ArgumentsTable->verticalHeader()->setVisible(false);
    m_ArgumentsTable->setFixedWidth(360);

    pPixmap = new QPixmap();
    pItem = new QGraphicsPixmapItem();
    pScene = new CGGraphicsScene();
    m_CGGraphicsView = new CGGraphicsView(this);

    m_CGVTKWidget = new CGVTKWidget(this);

    double clr[3];
    QColor defaultColor(25, 50, 75);
    VTKUtils::vtkColor(defaultColor, clr);
    m_CGVTKWidget->defaultRenderer()->SetBackground(clr);
    m_CGVTKWidget->showOrientationMarker();
    m_CGVTKWidget->update();

    m_StackedWidget = new QStackedWidget(this);
    m_StackedWidget->addWidget(m_CGGraphicsView);
    m_StackedWidget->addWidget(m_CGVTKWidget);

    QHBoxLayout *pArgumentsLayout = new QHBoxLayout();
    pArgumentsLayout->addWidget(m_ArgumentsTable);
    pArgumentsLayout->addWidget(m_StackedWidget);

    p2DShapeLineBtn = new QPushButton(this);
    p2DShapeRectBtn = new QPushButton(this);
    p2DShapeRotatedRectBtn = new QPushButton(this);
    p2DShapeCircleBtn = new QPushButton(this);
    p2DShapeConcentricCircleBtn = new QPushButton(this);
    p2DShapePolygonBtn = new QPushButton(this);
    p2DShapePolygonBtn->setVisible(false);
    p2DShapeExecuteBtn = new QPushButton(this);
    p2DShapeResetBtn = new QPushButton(this);

    p2DShapeLineBtn->setToolTip(tr(u8"直线"));
    p2DShapeRectBtn->setToolTip(tr(u8"矩形"));
    p2DShapeRotatedRectBtn->setToolTip(tr(u8"旋转矩形"));
    p2DShapeCircleBtn->setToolTip(tr(u8"圆"));
    p2DShapeConcentricCircleBtn->setToolTip(tr(u8"同心圆"));
    p2DShapePolygonBtn->setToolTip(tr(u8"多边形"));
    p2DShapeExecuteBtn->setToolTip(tr(u8"执行"));
    p2DShapeResetBtn->setToolTip(tr(u8"清除"));
    p2DShapeLineBtn->setFixedSize(QSize(50, 50));
    p2DShapeRectBtn->setFixedSize(QSize(50, 50));
    p2DShapeRotatedRectBtn->setFixedSize(QSize(50, 50));
    p2DShapeCircleBtn->setFixedSize(QSize(50, 50));
    p2DShapeConcentricCircleBtn->setFixedSize(QSize(50, 50));
    p2DShapePolygonBtn->setFixedSize(QSize(50, 50));
    p2DShapeExecuteBtn->setFixedSize(QSize(50, 50));
    p2DShapeResetBtn->setFixedSize(QSize(50, 50));
    p2DShapeLineBtn->setIcon(QIcon(":/res/icon/ShapeLine.png"));
    p2DShapeRectBtn->setIcon(QIcon(":/res/icon/ShapeRectangle.png"));
    p2DShapeRotatedRectBtn->setIcon(QIcon(":/res/icon/ShapeRotateRectangle.png"));
    p2DShapeCircleBtn->setIcon(QIcon(":/res/icon/ShapeCircle.png"));
    p2DShapeConcentricCircleBtn->setIcon(QIcon(":/res/icon/ShapeConcentricCircle.png"));
    p2DShapePolygonBtn->setIcon(QIcon(":/res/icon/ShapePolygon.png"));
    p2DShapeExecuteBtn->setIcon(QIcon(":/res/icon/ShapeExecute.png"));
    p2DShapeResetBtn->setIcon(QIcon(":/res/icon/ShapeClear.png"));
    p2DShapeLineBtn->setIconSize(QSize(30, 30));
    p2DShapeRectBtn->setIconSize(QSize(30, 30));
    p2DShapeRotatedRectBtn->setIconSize(QSize(30, 30));
    p2DShapeCircleBtn->setIconSize(QSize(30, 30));
    p2DShapeConcentricCircleBtn->setIconSize(QSize(30, 30));
    p2DShapePolygonBtn->setIconSize(QSize(30, 30));
    p2DShapeExecuteBtn->setIconSize(QSize(30, 30));
    p2DShapeResetBtn->setIconSize(QSize(30, 30));

    p2DShapeLayout = new QVBoxLayout();
    p2DShapeLayout->addWidget(p2DShapeLineBtn);
    p2DShapeLayout->addWidget(p2DShapeRectBtn);
    p2DShapeLayout->addWidget(p2DShapeRotatedRectBtn);
    p2DShapeLayout->addWidget(p2DShapeCircleBtn);
    p2DShapeLayout->addWidget(p2DShapeConcentricCircleBtn);
    p2DShapeLayout->addWidget(p2DShapePolygonBtn);
    p2DShapeLayout->addStretch();
    p2DShapeLayout->addWidget(p2DShapeExecuteBtn);
    p2DShapeLayout->addWidget(p2DShapeResetBtn);

    p3DShapeBoxBtn = new QPushButton(this);
    p3DShapeSphereBtn = new QPushButton(this);
    p3DShapePlaneBtn = new QPushButton(this);
    p3DShapeExecuteBtn = new QPushButton(this);
    p3DShapeResetBtn = new QPushButton(this);

    p3DShapeBoxBtn->setToolTip(tr(u8"包围盒"));
    p3DShapeSphereBtn->setToolTip(tr(u8"包围球"));
    p3DShapePlaneBtn->setToolTip(tr(u8"平面"));
    p3DShapeExecuteBtn->setToolTip(tr(u8"执行"));
    p3DShapeResetBtn->setToolTip(tr(u8"清除"));
    p3DShapeBoxBtn->setFixedSize(QSize(50, 50));
    p3DShapeSphereBtn->setFixedSize(QSize(50, 50));
    p3DShapePlaneBtn->setFixedSize(QSize(50, 50));
    p3DShapeExecuteBtn->setFixedSize(QSize(50, 50));
    p3DShapeResetBtn->setFixedSize(QSize(50, 50));
    p3DShapeBoxBtn->setIcon(QIcon(":/res/icon/ShapeBoundingBox.png"));
    p3DShapeSphereBtn->setIcon(QIcon(":/res/icon/ShapeBoundingSphere.png"));
    p3DShapePlaneBtn->setIcon(QIcon(":/res/icon/ShapePlane.png"));
    p3DShapeExecuteBtn->setIcon(QIcon(":/res/icon/ShapeExecute.png"));
    p3DShapeResetBtn->setIcon(QIcon(":/res/icon/ShapeClear.png"));
    p3DShapeBoxBtn->setIconSize(QSize(30, 30));
    p3DShapeSphereBtn->setIconSize(QSize(30, 30));
    p3DShapePlaneBtn->setIconSize(QSize(30, 30));
    p3DShapeExecuteBtn->setIconSize(QSize(30, 30));
    p3DShapeResetBtn->setIconSize(QSize(30, 30));

    p3DShapeLayout = new QVBoxLayout();
    p3DShapeLayout->addWidget(p3DShapeBoxBtn);
    p3DShapeLayout->addWidget(p3DShapeSphereBtn);
    p3DShapeLayout->addWidget(p3DShapePlaneBtn);
    p3DShapeLayout->addStretch();
    p3DShapeLayout->addWidget(p3DShapeExecuteBtn);
    p3DShapeLayout->addWidget(p3DShapeResetBtn);

    m_2DShapeWidget = new QWidget(this);
    m_3DShapeWidget = new QWidget(this);
    m_2DShapeWidget->setLayout(p2DShapeLayout);
    m_3DShapeWidget->setLayout(p3DShapeLayout);

    QHBoxLayout *pMainLayout = new QHBoxLayout();
    pMainLayout->addLayout(pArgumentsLayout);
    pMainLayout->addWidget(m_2DShapeWidget);
    pMainLayout->addWidget(m_3DShapeWidget);
    setLayout(pMainLayout);
}

void NodeBlockWidget::InitConnections()
{
    connect(m_ArgumentsTable, &QTableWidget::itemChanged, this, &NodeBlockWidget::OnTableWidgetItemChanged);

    connect(p2DShapeLineBtn, &QPushButton::clicked, this, [&]{
            RemoveShapeItem();
            pScene->addItem(m_LineItem);
            pScene->update();
            m_CGGraphicsView->RemoveFilter();
            m_CurrentShapeType = ItemType::Line;
            IsShapeItem = true;
    });
    connect(p2DShapeRectBtn, &QPushButton::clicked, this, [&]{
            RemoveShapeItem();
            pScene->addItem(m_RectItem);
            pScene->update();
            m_CGGraphicsView->RemoveFilter();
            m_CurrentShapeType = ItemType::Rectangle;
            IsShapeItem = true;
    });
    connect(p2DShapeRotatedRectBtn, &QPushButton::clicked, this, [&]{
            RemoveShapeItem();
            pScene->addItem(m_RotateRectangleItem);
            m_CGGraphicsView->RemoveFilter();
            m_CurrentShapeType = ItemType::RotateRectangle;
            IsShapeItem = true;
    });
    connect(p2DShapeCircleBtn, &QPushButton::clicked, this, [&]{
            RemoveShapeItem();
            pScene->addItem(m_CircleItem);
            m_CGGraphicsView->RemoveFilter();
            m_CurrentShapeType = ItemType::Circle;
            IsShapeItem = true;
    });
    connect(p2DShapeConcentricCircleBtn, &QPushButton::clicked, this, [&]{
            RemoveShapeItem();
            pScene->addItem(m_ConcentricCircleItem);
            m_CGGraphicsView->RemoveFilter();
            m_CurrentShapeType = ItemType::ConcentricCircle;
            IsShapeItem = true;
    });
    connect(p2DShapePolygonBtn, &QPushButton::clicked, this, [&]{
            RemoveShapeItem();
            pScene->addItem(m_PolygonItem);
            m_CGGraphicsView->RemoveFilter();
            m_CurrentShapeType = ItemType::Polygon;
            connect(pScene, &CGGraphicsScene::updatePoint, m_PolygonItem, &CGShapePolygonItem::pushPoint);
            IsShapeItem = true;
    });

    connect(p2DShapeExecuteBtn, &QPushButton::clicked, this, [&]{
            QString msg = ShapeItemValue();
            emit SignalShapeItemValue(msg);
    });
    connect(p2DShapeResetBtn, &QPushButton::clicked, this, [&]{
            RemoveShapeItem();
            m_CGGraphicsView->InstallFilter();
            IsShapeItem = false;
            emit SignalShapeItemValue("");
    });


    connect(p3DShapeBoxBtn, &QPushButton::clicked, this, [&]{
            RemoveShapeItem();
            InitBoxWidgetTools();
            m_CurrentShapeType = ItemType::BoundingBox;
            IsShapeItem = true;
    });
    connect(p3DShapeSphereBtn, &QPushButton::clicked, this, [&]{
            RemoveShapeItem();
            InitSphereWidgetTools();
            m_CurrentShapeType = ItemType::BoundingSphere;
            IsShapeItem = true;
    });
    connect(p3DShapePlaneBtn, &QPushButton::clicked, this, [&]{
            RemoveShapeItem();
            InitPlaneWidgetTools();
            m_CurrentShapeType = ItemType::Plane;
            IsShapeItem = true;
    });

    connect(p3DShapeExecuteBtn, &QPushButton::clicked, this, [&]{
            QString msg = ShapeItemValue();
            emit SignalShapeItemValue(msg);
    });
    connect(p3DShapeResetBtn, &QPushButton::clicked, this, [&]{
            RemoveShapeItem();
            IsShapeItem = false;
            emit SignalShapeItemValue("");
    });
}

void NodeBlockWidget::InitTableWidget()
{
    m_ArgumentsTable->setColumnCount(3);
    m_ArgumentsTable->setColumnWidth(0, 60);
    m_ArgumentsTable->setColumnWidth(1, 160);
    m_ArgumentsTable->setColumnWidth(2, 120);
    m_ArgumentsTable->setRowCount(16);
    m_ArgumentsTable->setHorizontalHeaderLabels(QStringList() << QString::fromLocal8Bit("序号")
                                                              << QString::fromLocal8Bit("参数")
                                                              << QString::fromLocal8Bit("数值"));
}

void NodeBlockWidget::InitShapeItems()
{
    m_LineItem = new CGShapeLineItem(64, 64, 512, 512);
    m_RectItem = new CGShapeRectItem(64, 64, 512, 512);
    m_RotateRectangleItem = new CGShapeRotateRectangleItem(128, 128, 512, 512, 0);
    m_CircleItem = new CGShapeCircleItem(512, 512, 256);
    m_ConcentricCircleItem = new CGShapeConcentricCircleItem(512, 512, 128, 256);
    m_PolygonItem = new CGShapePolygonItem();
}

void NodeBlockWidget::InitPlaneWidgetTools()
{
    CGVTKUtils::vtkInitOnce(m_pPlaneWidgetTool);
    m_CGPlaneWidgeter = new CGVTKUtils::CGPlaneWidgetObserver();
    connect(m_CGPlaneWidgeter, &CGVTKUtils::CGPlaneWidgetObserver::planesChanged, this, &NodeBlockWidget::ToolPlaneWidgetValue);

    m_pPlaneWidgetTool->AddObserver(vtkCommand::EndInteractionEvent, m_CGPlaneWidgeter);
    m_pPlaneWidgetTool->SetInteractor((m_CGVTKWidget->GetInteractor()));
    m_pPlaneWidgetTool->SetProp3D(_Actor);
    m_pPlaneWidgetTool->GetPlaneProperty()->SetColor(1, 0, 1);
    m_pPlaneWidgetTool->GetPlaneProperty()->SetOpacity(0.7);
    m_pPlaneWidgetTool->GetPlaneProperty()->SetLineWidth(3);
    m_pPlaneWidgetTool->SetRepresentationToSurface();
    m_pPlaneWidgetTool->NormalToZAxisOn();
    m_pPlaneWidgetTool->PlaceWidget();
    m_pPlaneWidgetTool->On();
}

void NodeBlockWidget::InitBoxWidgetTools()
{
    CGVTKUtils::vtkInitOnce(m_pBoxWidgetTool);
    m_CGBoxWidgeter = new CGVTKUtils::CGBoxWidgetObserver();
    connect(m_CGBoxWidgeter, &CGVTKUtils::CGBoxWidgetObserver::planesChanged, this, &NodeBlockWidget::ToolBoxWidgetValue);

    m_pBoxWidgetTool->AddObserver(vtkCommand::EndInteractionEvent, m_CGBoxWidgeter);
    m_pBoxWidgetTool->SetPlaceFactor(1.0);
    m_pBoxWidgetTool->SetRotationEnabled(0);
    m_pBoxWidgetTool->SetInteractor(m_CGVTKWidget->GetInteractor());
    m_pBoxWidgetTool->SetProp3D(_Actor);
    m_pBoxWidgetTool->GetSelectedFaceProperty()->SetColor(1, 0, 1);
    m_pBoxWidgetTool->GetSelectedFaceProperty()->SetOpacity(0.7);
    m_pBoxWidgetTool->PlaceWidget();
    m_pBoxWidgetTool->On();
}

void NodeBlockWidget::InitSphereWidgetTools()
{
    CGVTKUtils::vtkInitOnce(m_pSphereWidgetTool);
    m_CGSphereWidgeter = new CGVTKUtils::CGSphereWidgetObserver();
    connect(m_CGSphereWidgeter, &CGVTKUtils::CGSphereWidgetObserver::sphereChanged, this, &NodeBlockWidget::ToolSphereWidgetValue);

    m_pSphereWidgetTool->AddObserver(vtkCommand::EndInteractionEvent, m_CGSphereWidgeter);
    m_pSphereWidgetTool->SetPlaceFactor(1.0);
    m_pSphereWidgetTool->SetThetaResolution(36);
    m_pSphereWidgetTool->SetPhiResolution(36);
    m_pSphereWidgetTool->GetSphereProperty()->SetColor(1, 0, 0.7);
    m_pSphereWidgetTool->GetSphereProperty()->SetOpacity(0.3);
    m_pSphereWidgetTool->SetRepresentationToSurface();
    m_pSphereWidgetTool->SetInteractor(m_CGVTKWidget->GetInteractor());
    m_pSphereWidgetTool->SetProp3D(_Actor);
    m_pSphereWidgetTool->GetSelectedSphereProperty()->SetColor(0, 1, 0);
    m_pSphereWidgetTool->GetSelectedSphereProperty()->SetOpacity(0.7);
    m_pSphereWidgetTool->PlaceWidget();
    m_pSphereWidgetTool->On();
}

void NodeBlockWidget::ClearImage()
{
    if (bGraphicsScene)
        m_CGGraphicsView->RemoveALLItems();
}

void NodeBlockWidget::ClearPointCloud()
{
    int num = m_CGVTKWidget->actors().count();
    for (int i = 0; i < num; ++i)
    {
        m_CGVTKWidget->defaultRenderer()->RemoveActor(m_CGVTKWidget->actors()[i]);
    }
}

void NodeBlockWidget::RemoveShapeItem()
{
    if (IsShapeItem)
    {
        switch (m_CurrentShapeType)
        {
        case ItemType::Line:
            pScene->removeItem(m_LineItem);
            break;
        case ItemType::Rectangle:
            pScene->removeItem(m_RectItem);
            break;
        case ItemType::RotateRectangle:
            pScene->removeItem(m_RotateRectangleItem);
            break;
        case ItemType::Circle:
            pScene->removeItem(m_CircleItem);
            break;
        case ItemType::ConcentricCircle:
            pScene->removeItem(m_ConcentricCircleItem);
            break;
        case ItemType::Polygon:
            pScene->removeItem(m_PolygonItem);
            disconnect(pScene, &CGGraphicsScene::updatePoint, m_PolygonItem, &CGShapePolygonItem::pushPoint);
            break;
        case ItemType::BoundingBox:
            m_pBoxWidgetTool->Off();
            m_CGVTKWidget->GetInteractor()->RemoveObserver(m_CGBoxWidgeter);
            break;
        case ItemType::BoundingSphere:
            m_pSphereWidgetTool->Off();
            m_CGVTKWidget->GetInteractor()->RemoveObserver(m_CGSphereWidgeter);
            break;
        case ItemType::Plane:
            m_pPlaneWidgetTool->Off();
            m_CGVTKWidget->GetInteractor()->RemoveObserver(m_CGPlaneWidgeter);
            break;
        default:
            break;
        }
        pScene->update();
        m_CGGraphicsView->InstallFilter();

        IsShapeItem = false;
    }
}

QString NodeBlockWidget::ShapeItemValue()
{
    QString value;
    CGShapeLine line;
    CGShapeRectangle rect;
    CGShapeRotateRectangle rrect;
    CGShapeCircle circle;
    CGShapeConcentricCircle ccircle;

    if (IsShapeItem)
    {
        switch (m_CurrentShapeType)
        {
        case ItemType::Line:
            m_LineItem->GetLine(line);
            value = QString("Line  X1:%1  Y1:%2  X2:%3  Y2:%4").arg(line.p1_x).arg(line.p1_y).arg(line.p2_x).arg(line.p2_y);
            break;
        case ItemType::Rectangle:
            m_RectItem->GetRect(rect);
            value = QString("Rectangle  X:%1  Y:%2  W:%3  H:%4").arg(rect.col).arg(rect.row).arg(rect.width).arg(rect.height);
            break;
        case ItemType::RotateRectangle:
            m_RotateRectangleItem->GetRotateRect(rrect);
            value = QString("RotateRectangle  X:%1  Y:%2  Phi:%3  W:%4  H:%5").arg(rrect.col).arg(rrect.row).arg(rrect.phi).arg(rrect.lenth1).arg(rrect.lenth2);
            break;
        case ItemType::Circle:
            m_CircleItem->GetCircle(circle);
            value = QString("Circle  X:%1  Y:%2  R:%3").arg(circle.col).arg(circle.row).arg(circle.radius);
            break;
        case ItemType::ConcentricCircle:
            m_ConcentricCircleItem->GetConcentricCircle(ccircle);
            value = QString("ConcentricCircle  X:%1  Y:%2  Rmin:%3  Rmax:%4").arg(ccircle.col).arg(ccircle.row).arg(ccircle.small_radius).arg(ccircle.big_radius);
            break;
        case ItemType::Polygon:
            value = QString("Polygon  ");
            break;
        case ItemType::BoundingBox:
            value = QString("Box  ").append(StrToolBoxWidgetValue);
            break;
        case ItemType::BoundingSphere:
            value = QString("Sphere  ").append(StrToolSphereWidgetValue);
            break;
        case ItemType::Plane:
            value = QString("Plane  ").append(StrToolPlaneWidgetValue);
            break;
        default:
            break;
        }
        OnSetAlgorithmArguments(value);
    }
    return value;
}

QString NodeBlockWidget::ToolBoxWidgetValue(vtkPlanes *planes)
{
    double bounds[6];
    planes->GetPoints()->GetBounds(bounds);
    double Xmin = bounds[0]; double Xmax = bounds[1];
    double Ymin = bounds[2]; double Ymax = bounds[3];
    double Zmin = bounds[4]; double Zmax = bounds[5];

    QString value;
    value = QString("Xmin:%1  Xmax:%2  Ymin:%3  Ymax:%4  Zmin:%5  Zmax:%6").arg(Xmin).arg(Xmax).arg(Ymin).arg(Ymax).arg(Zmin).arg(Zmax);
    StrToolBoxWidgetValue = value;
    return value;
}

QString NodeBlockWidget::ToolSphereWidgetValue(double *sphere)
{
    double X = sphere[0]; double Y = sphere[1]; double Z = sphere[2]; double R = sphere[3];
    QString value;
    value = QString("X:%1  Y:%2  Z:%3  Radius:%4").arg(X).arg(Y).arg(Z).arg(R);
    StrToolSphereWidgetValue = value;
    return value;
}

QString NodeBlockWidget::ToolPlaneWidgetValue(vtkPlane *plane)
{
    double normal[3]; double origin[3];
    plane->GetNormal(normal);
    plane->GetOrigin(origin);
    double A = normal[0]; double B = normal[1]; double C = normal[2];
    double X = origin[0]; double Y = origin[1]; double Z = origin[2];

    QString value;
    value = QString("A:%1  B:%2  C:%3  X:%4  Y:%5  Z:%6").arg(A).arg(B).arg(C).arg(X).arg(Y).arg(Z);
    StrToolPlaneWidgetValue = value;
    return value;
}

void NodeBlockWidget::PointCloud2VTKActor(PointCloudT::Ptr cloud, vtkActor *actor)
{
    if (cloud->empty()) return;
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

    long long n = 0;
    double x, y, z;
    double r, g, b;
    double MinZ = min_pt.z, MaxZ = max_pt.z;
    for (size_t i = 0; i < cloud->size(); ++i)
    {

        n = (double)i;
        x = (double)cloud->points[i].x;
        y = (double)cloud->points[i].y;
        z = (double)cloud->points[i].z;
        r = (double)cloud->points[i].r;
        g = (double)cloud->points[i].g;
        b = (double)cloud->points[i].b;

        points->InsertPoint(n, x, y, z);
     }

     vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
     polyData->SetPoints(points);

     vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
     glyphFilter->SetInputData(polyData);
     glyphFilter->Update();

     vtkSmartPointer<vtkElevationFilter> coloredGrid = vtkElevationFilter::New();
     coloredGrid->SetInputConnection(glyphFilter->GetOutputPort());
     coloredGrid->SetLowPoint(0, 0, MaxZ);
     coloredGrid->SetHighPoint(0, 0, MinZ);

     vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
     mapper->SetInputConnection(coloredGrid->GetOutputPort());

     actor->SetMapper(mapper);
}

void NodeBlockWidget::OnTableWidgetItemChanged(QTableWidgetItem *current)
{
    if (current == nullptr) return;
    int row = current->row();
    float value = 0.0f;

    if (m_plugin->GetAlgorithmArguments().size() == 0) return;
    if (m_ArgumentsTable->item(row, 0) == nullptr ||
        m_ArgumentsTable->item(row, 1) == nullptr ||
        m_ArgumentsTable->item(row, 2) == nullptr) return;

    QString strValue = m_ArgumentsTable->item(row, 2)->text().trimmed();
    value = strValue.toFloat();

    m_args[row].VALUE = value;
}

void NodeBlockWidget::OnSendAlgorithmArguments()
{
    m_plugin->SetAlgorithmArguments(m_args);

    for (auto it = m_args.begin(); it != m_args.end(); ++it)
    {
        m_block->m_NodeItem->m_Parameters[it->ARG] = it->VALUE;
    }
}

void NodeBlockWidget::OnSetAlgorithmArguments(const QString value)
{
    QStringList strList = value.split(QRegExp("  "));
    QStringList strListValue;
    for (int i = 1; i < strList.size(); ++i)
    {
        QString strCell = strList.at(i);
        QStringList strValue = strCell.split(QRegExp(":")); 
        strListValue << strValue.at(1);
    }
    //qDebug() << strListValue;


    switch (m_CurrentShapeType)
    {
        case ItemType::Line:
            for (int i = 1; i < m_args.size(); i++)
            {
                QString arg = m_ArgumentsTable->item(i, 1)->text();
                //qDebug() << arg;
                if (arg == NULL) continue;
                if (arg.contains("X1")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(0)); }
                if (arg.contains("Y1")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(1)); }
                if (arg.contains("X2")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(2)); }
                if (arg.contains("Y2")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(3)); }
            }
        break;

        case ItemType::Rectangle:
            for (int i = 0; i < m_args.size(); i++)
            {
                QString arg = m_ArgumentsTable->item(i, 1)->text();
                //qDebug() << arg;
                if (arg == NULL) continue;
                if (arg.contains("X")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(0)); }
                if (arg.contains("Y")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(1)); }
                if (arg.contains("Width")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(2)); }
                if (arg.contains("Height")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(3)); }
            }
        break;

        case ItemType::RotateRectangle:
        for (int i = 0; i < m_args.size(); i++)
        {
            QString arg = m_ArgumentsTable->item(i, 1)->text();
            //qDebug() << arg;
            if (arg == NULL) continue;
            if (arg.contains("X")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(0)); }
            if (arg.contains("Y")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(1)); }
            if (arg.contains("Phi")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(2)); }
            if (arg.contains("Width")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(3)); }
            if (arg.contains("Height")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(4)); }
        }
        break;

        case ItemType::Circle:
        for (int i = 0; i < m_args.size(); i++)
        {
            QString arg = m_ArgumentsTable->item(i, 1)->text();
            //qDebug() << arg;
            if (arg == NULL) continue;
            if (arg.contains("X")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(0)); }
            if (arg.contains("Y")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(1)); }
            if (arg.contains("Radius")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(2)); }
        }
        break;

        case ItemType::ConcentricCircle:
        for (int i = 0; i < m_args.size(); i++)
        {
            QString arg = m_ArgumentsTable->item(i, 1)->text();
            //qDebug() << arg;
            if (arg == NULL) continue;
            if (arg.contains("X")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(0)); }
            if (arg.contains("Y")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(1)); }
            if (arg.contains("Rmin")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(2)); }
            if (arg.contains("Rmax")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(3)); }
        }
        break;

        case ItemType::Polygon:
        for (int i = 0; i < m_args.size(); i++)
        {
            QString arg = m_ArgumentsTable->item(i, 1)->text();
            //qDebug() << arg;
            if (arg == NULL) continue;
        }
        break;

        case ItemType::BoundingBox:
        for (int i = 0; i < m_args.size(); i++)
        {
            QString arg = m_ArgumentsTable->item(i, 1)->text();
            //qDebug() << arg;
            if (arg == NULL) continue;
            if (arg.contains("Xmin")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(0)); }
            if (arg.contains("Xmax")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(1)); }
            if (arg.contains("Ymin")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(2)); }
            if (arg.contains("Ymax")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(3)); }
            if (arg.contains("Zmin")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(4)); }
            if (arg.contains("Zmax")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(5)); }
        }
        break;

        case ItemType::BoundingSphere:
        for (int i = 0; i < m_args.size(); i++)
        {
            QString arg = m_ArgumentsTable->item(i, 1)->text();
            //qDebug() << arg;
            if (arg == NULL) continue;
            if (arg.contains("X")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(0)); }
            if (arg.contains("Y")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(1)); }
            if (arg.contains("Z")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(2)); }
            if (arg.contains("R")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(3)); }
        }
        break;

        case ItemType::Plane:
        for (int i = 0; i < m_args.size(); i++)
        {
            QString arg = m_ArgumentsTable->item(i, 1)->text();
            //qDebug() << arg;
            if (arg == NULL) continue;
            if (arg.contains("A")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(0)); }
            if (arg.contains("B")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(1)); }
            if (arg.contains("C")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(2)); }
            if (arg.contains("X")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(3)); }
            if (arg.contains("Y")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(4)); }
            if (arg.contains("Z")) { m_ArgumentsTable->item(i, 2)->setText(strListValue.at(5)); }
        }
        break;

        default:
        break;
    }
}
