#include "NodeBlockWidget.h"
#include <QLabel>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QDebug>
#include <CGVTKHeader.h>
#include <CGVTKUtils.h>
#include <CGVTKWidget.h>
#include <CGImageFormatConvert.h>
#include <vtkImageData.h>
#include <vtkImageActor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkInteractorStyleTrackballCamera.h>


NodeBlockWidget *NodeBlockWidget::m_NodeBlockWidget = nullptr;

NodeBlockWidget::NodeBlockWidget(QWidget *parent) : QWidget(parent)
{
    InitUi();
    InitConnections();
    InitTableWidget();

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
        m_ArgumentsTable->setItem(i, 2, new QTableWidgetItem(QString::number(args.at(i).VALUE, 'f', 3)));

        m_ArgumentsTable->item(i, 0)->setFlags(Qt::ItemIsEnabled);
        m_ArgumentsTable->item(i, 1)->setFlags(Qt::ItemIsEnabled);
    }
    m_ArgumentsTable->update();
}

void NodeBlockWidget::LoadAlgorithmShowData(CG_SHOWDATA &data)
{
    if (data.Type == CG_ALGORITHM_TYPE::ALG2D)
    {
        if (data.Data.canConvert<cv::Mat>())
            _image = data.Data.value<cv::Mat>();
        QImage _Image = CG::CVMat2QImage(_image);

        vtkSmartPointer<vtkImageData> _data = vtkSmartPointer<vtkImageData>::New();
        CGVTKUtils::qImageToVtkImage(_Image, _data);
        vtkSmartPointer<vtkImageActor> _actor = vtkSmartPointer<vtkImageActor>::New();
        _actor->SetInputData(_data);
        ClearPointCloud();
        m_CGVTKWidget->addActor(_actor, QColor(25, 50, 75));

        vtkSmartPointer<vtkInteractorStyleImage> style = vtkSmartPointer<vtkInteractorStyleImage>::New();
        style->SetDefaultRenderer(m_CGVTKWidget->defaultRenderer());

        m_CGVTKWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(style);
        m_CGVTKWidget->defaultRenderer()->GetActiveCamera()->SetPosition(0, 0, 1);
        m_CGVTKWidget->defaultRenderer()->GetActiveCamera()->SetFocalPoint(0, 0, 0);
        m_CGVTKWidget->defaultRenderer()->GetActiveCamera()->SetViewUp(0, 1, 0);
        m_CGVTKWidget->defaultRenderer()->ResetCamera();
        m_CGVTKWidget->update();
    }

    if (data.Type == CG_ALGORITHM_TYPE::ALG3D)
    {
        if (data.Data.canConvert<PointCloudT::Ptr>())
            _cloud = data.Data.value<PointCloudT::Ptr>();

        vtkSmartPointer<vtkActor> _actor = vtkSmartPointer<vtkActor>::New();
        PointCloud2VTKActor(_cloud, _actor);
        ClearPointCloud();
        m_CGVTKWidget->addActor(_actor, QColor(25, 50, 75));

        vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
        style->SetDefaultRenderer(m_CGVTKWidget->defaultRenderer());

        m_CGVTKWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(style);
        m_CGVTKWidget->defaultRenderer()->ResetCamera();
        m_CGVTKWidget->update();
    }
}

void NodeBlockWidget::SetCurrentAlgorithmPlugin(AlgorithmInterface *plugin)
{
    m_plugin = plugin;
    m_args = plugin->GetAlgorithmArguments();
}

void NodeBlockWidget::InitUi()
{
    m_ArgumentsTable = new QTableWidget(this);
    m_ArgumentsTable->setShowGrid(true);
    m_ArgumentsTable->verticalHeader()->setVisible(false);
    m_ArgumentsTable->setFixedWidth(360);

    m_CGVTKWidget = new CGVTKWidget(this);

    double clr[3];
    QColor defaultColor(25, 50, 75);
    VTKUtils::vtkColor(defaultColor, clr);
    m_CGVTKWidget->defaultRenderer()->SetBackground(clr);
    m_CGVTKWidget->showOrientationMarker();
    m_CGVTKWidget->update();

    QHBoxLayout *pArgumentsLayout = new QHBoxLayout();
    pArgumentsLayout->addWidget(m_ArgumentsTable);
    pArgumentsLayout->addWidget(m_CGVTKWidget);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addLayout(pArgumentsLayout);
    setLayout(pMainLayout);
}

void NodeBlockWidget::InitConnections()
{
    connect(m_ArgumentsTable, &QTableWidget::itemChanged, this, &NodeBlockWidget::OnTableWidgetItemChanged);
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

void NodeBlockWidget::ClearPointCloud()
{
    int num = m_CGVTKWidget->actors().count();
    for (int i = 0; i < num; ++i)
    {
        m_CGVTKWidget->defaultRenderer()->RemoveActor(m_CGVTKWidget->actors()[i]);
    }
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

void NodeBlockWidget::OnSendAlgorithmArguuments()
{
    m_plugin->SetAlgorithmArguments(m_args);
}
