#include "CG3DImageView.h"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>
#include <QDebug>
#include <Utils.h>
#include <CGVTKUtils.h>
#include <CGVTKHeader.h>
#include <CGPCLHeader.h>
#include <CGPointCloud.h>
#include <vtkDistanceWidget.h>
#include <vtkDistanceRepresentation3D.h>
#include <vtkAngleWidget.h>
#include <vtkAngleRepresentation3D.h>
#include <vtkBoxWidget.h>


CG3DImageView::CG3DImageView(QWidget *parent) : CGBaseWidget(parent)
{
    InitUi();
    InitConnections();
    InitTools();
    setWindowTitle(tr(u8"3D  图像"));
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
}

CG3DImageView::~CG3DImageView()
{

}

void CG3DImageView::OnUseTool()
{
    if (g_PointCloud == nullptr) return;

    RemoveTools();

    switch (m_CurrentToolType)
    {
    case DistanceTool:
        InitDistanceTool();
        break;

    case AngleTool:
        InitAngleTool();
        break;

    case BoxTool:
        InitBoxTool();
        break;

    default:
        break;
    }
    m_LastToolType = m_CurrentToolType;
    m_CGVTKWidget->update();

    IsTool = true;
}

void CG3DImageView::OnDelTool()
{
    RemoveTools();
}

void CG3DImageView::InitUi()
{
    m_CGVTKWidget = new CGVTKWidget(this);

    double clr[3];
    QColor defaultColor(25, 50, 75);
    VTKUtils::vtkColor(defaultColor, clr);
    m_CGVTKWidget->defaultRenderer()->SetBackground(clr);
    m_CGVTKWidget->showOrientationMarker();
    m_CGVTKWidget->update();

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_CGVTKWidget);

    setLayout(pMainLayout);

    CGVTKUtils::vtkInitOnce(m_Actor);
    GetCamera();
}

void CG3DImageView::InitConnections()
{

}

void CG3DImageView::LoadPCD(const std::string filename)
{
    //! qDebug() << "LoadPCD " << QString::fromStdString(filename);
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    CG::LoadPCDFile(filename, actor);

    m_Actor = actor;
    m_CGVTKWidget->addActor3D(actor, QColor(25, 50, 75));
    m_CGVTKWidget->defaultRenderer()->ResetCamera();
    m_CGVTKWidget->update();
}

void CG3DImageView::LoadCSV(const std::string filename)
{
    //! qDebug() << "LoadCSV " << QString::fromStdString(filename);
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    CG::LoadCSVFile(filename, actor);

    m_Actor = actor;
    m_CGVTKWidget->addActor3D(actor, QColor(25, 50, 75));
    m_CGVTKWidget->defaultRenderer()->ResetCamera();
    m_CGVTKWidget->update();
}

void CG3DImageView::LoadTXT(const std::string filename)
{
    //!qDebug() << "LoadTXT " << QString::fromStdString(filename);
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    CG::LoadTXTFile(filename, actor);

    m_Actor = actor;
    m_CGVTKWidget->addActor3D(actor, QColor(25, 50, 75));
    m_CGVTKWidget->defaultRenderer()->ResetCamera();
    m_CGVTKWidget->update();
}

void CG3DImageView::ClearPointCloud()
{
    int num = m_CGVTKWidget->actors3d().count();
    for (int i = 0; i < num; ++i)
    {
        m_CGVTKWidget->defaultRenderer()->RemoveActor(m_CGVTKWidget->actors3d()[i]);
    }
    m_CGVTKWidget->update();
}

void CG3DImageView::ResetCameraParameter()
{
    m_CGVTKCamera->SetFocalPoint(pCameraFocalPoint);
    m_CGVTKCamera->SetPosition(pCameraPosition);
    m_CGVTKCamera->SetViewUp(pCameraViewUp);
    m_CGVTKCamera->SetClippingRange(pCameraClippingRange);
    m_CGVTKCamera->SetViewAngle(fovy);
}

void CG3DImageView::GetCameraParameter()
{
    GetCameraFocalPoint();
    GetCameraPosition();
    GetCameraViewUp();
    GetCameraClippingRange();
    GetCameraViewAngle();
}

void CG3DImageView::SetCameraParameter(double pos_x, double pos_y, double pos_z, double up_x, double up_y, double up_z)
{
    m_CGVTKCamera->SetPosition(pos_x, pos_y, pos_z);
    m_CGVTKCamera->SetViewUp(up_x, up_y, up_z);
}

vtkCamera* CG3DImageView::GetCamera()
{
    m_CGVTKCamera = m_CGVTKWidget->defaultRenderer()->GetActiveCamera();
    GetCameraParameter();
    return m_CGVTKCamera;
}

double *CG3DImageView::GetCameraFocalPoint()
{
    static double arg[3];
    m_CGVTKWidget->defaultRenderer()->GetActiveCamera()->GetFocalPoint(arg);
    pCameraFocalPoint = arg;
    return pCameraFocalPoint;
}

double *CG3DImageView::GetCameraPosition()
{
    static double arg[3];
    m_CGVTKWidget->defaultRenderer()->GetActiveCamera()->GetPosition(arg);
    pCameraPosition = arg;
    return pCameraPosition;
}

double *CG3DImageView::GetCameraViewUp()
{
    static double arg[3];
    m_CGVTKWidget->defaultRenderer()->GetActiveCamera()->GetViewUp(arg);
    pCameraViewUp = arg;
    return pCameraViewUp;
}

double *CG3DImageView::GetCameraClippingRange()
{
    static double arg[2];
    m_CGVTKWidget->defaultRenderer()->GetActiveCamera()->GetClippingRange(arg);
    pCameraClippingRange = arg;
    return pCameraClippingRange;
}

double CG3DImageView::GetCameraViewAngle()
{
    fovy = m_CGVTKWidget->defaultRenderer()->GetActiveCamera()->GetViewAngle();
    return fovy;
}

void CG3DImageView::InitDistanceTool()
{
    m_pDistanceWidgetTool->SetRepresentation(m_pDistanceRep);
    m_pDistanceWidgetTool->SetInteractor(m_CGVTKWidget->GetInteractor());
    m_pDistanceWidgetTool->On();
}

void CG3DImageView::InitAngleTool()
{
    m_pAngleWidgetTool->SetRepresentation(m_pAngleRep);
    m_pAngleWidgetTool->SetInteractor(m_CGVTKWidget->GetInteractor());
    m_pAngleWidgetTool->On();
}

void CG3DImageView::InitBoxTool()
{
    m_pBoxWidgetTool->SetInteractor(m_CGVTKWidget->GetInteractor());
    m_pBoxWidgetTool->SetPlaceFactor(1.0);
    m_pBoxWidgetTool->SetProp3D(m_Actor);
    m_pBoxWidgetTool->PlaceWidget();
    m_pBoxWidgetTool->On();
}

vtkActor* CG3DImageView::GetActor() const
{
    return m_CGVTKWidget->actors3d().back();
}

void CG3DImageView::InitTools()
{
    CGVTKUtils::vtkInitOnce(m_pDistanceWidgetTool);
    CGVTKUtils::vtkInitOnce(m_pDistanceRep);
    CGVTKUtils::vtkInitOnce(m_pAngleWidgetTool);
    CGVTKUtils::vtkInitOnce(m_pAngleRep);
    CGVTKUtils::vtkInitOnce(m_pBoxWidgetTool);
}

void CG3DImageView::RemoveTools()
{
    if (IsTool)
    {
        switch (m_LastToolType)
        {
        case DistanceTool:
            m_pDistanceWidgetTool->Off();
            break;

        case AngleTool:
            m_pAngleWidgetTool->Off();
            break;

        case BoxTool:
            m_pBoxWidgetTool->Off();
            break;
        default:
            break;
        }
        m_CGVTKWidget->update();

        IsTool = false;
    }
}
