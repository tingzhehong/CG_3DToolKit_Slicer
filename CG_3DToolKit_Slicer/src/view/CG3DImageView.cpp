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
#include <CGOCVHeader.h>
#include <CGPointCloud.h>
#include <vtkDistanceWidget.h>
#include <vtkDistanceRepresentation3D.h>
#include <vtkAngleWidget.h>
#include <vtkAngleRepresentation3D.h>
#include <vtkBoxWidget.h>
#include <vtkPlaneWidget.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkPointPicker.h>
#include <vtkSphereSource.h>
#include <vtkLineSource.h>
#include <vtkCubeAxesActor.h>
#include <vtkAppendPolyData.h>
#include <vtkFollower.h>

static QStack<CG_Point> s_PickPointsStack;

CG3DImageView::CG3DImageView(QWidget *parent) : CGBaseWidget(parent)
{
    InitUi();
    InitActors();
    InitTools();
    InitPointPick();
    InitConnections();
    GetCamera();
    ShowText2D();
    ShowText3D();
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

    case PlaneTool:
        InitPlaneTool();
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

void CG3DImageView::OnUpdatePoint(float x, float y, float z)
{
    QString chrX = QString::asprintf("%.4f", x);
    QString chrY = QString::asprintf("%.4f", y);
    QString chrZ = QString::asprintf("%.4f", z);
    std::string strX, strY, strZ;
    strX = "X: "; strX.append(chrX.toStdString());
    strY = "Y: "; strY.append(chrY.toStdString());
    strZ = "Z: "; strZ.append(chrZ.toStdString());
    m_TextActor_X->SetInput(strX.c_str());
    m_TextActor_Y->SetInput(strY.c_str());
    m_TextActor_Z->SetInput(strZ.c_str());

    switch (m_PickType)
    {
    case SinglePoint:
        HandlePickPointCoordinate(x, y, z);
        break;
    case DoublePoint:
        HandlePickPointDistance(x, y, z);
        break;
    default:
        break;
    }

    m_CGVTKWidget->update();
}

void CG3DImageView::OnBoxWidgetPlaneChanged(vtkPlanes *planes)
{
    CGImage3DGraphicsItemAdapter::getInstance()->SendPlanes(planes);
}

void CG3DImageView::OnPlaneWidgetPlaneChanged(vtkPlane *plane)
{
    CGImage3DGraphicsItemAdapter::getInstance()->SendPlane(plane);
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
}

void CG3DImageView::InitConnections()
{
    connect(m_CGPointPicker, &CGVTKUtils::CGPointPickObserver::SignalPoint, this, &CG3DImageView::OnUpdatePoint);
}

void CG3DImageView::ShowText2D()
{
    m_TextActor_X->SetVisibility(0);
    m_TextActor_Y->SetVisibility(0);
    m_TextActor_Z->SetVisibility(0);
    m_TextActor_Distance->SetVisibility(0);

    m_TextActor_X->SetPosition(50, 700);
    m_TextActor_X->GetTextProperty()->SetFontSize(18);
    m_TextActor_X->GetTextProperty()->SetColor(1, 1, 0);

    m_TextActor_Y->SetPosition(50, 680);
    m_TextActor_Y->GetTextProperty()->SetFontSize(18);
    m_TextActor_Y->GetTextProperty()->SetColor(1, 1, 0);

    m_TextActor_Z->SetPosition(50, 660);
    m_TextActor_Z->GetTextProperty()->SetFontSize(18);
    m_TextActor_Z->GetTextProperty()->SetColor(1, 1, 0);

    m_TextActor_Distance->SetPosition(50, 640);
    m_TextActor_Distance->GetTextProperty()->SetFontSize(18);
    m_TextActor_Distance->GetTextProperty()->SetColor(1, 1, 1);

    m_CGVTKWidget->defaultRenderer()->AddActor(m_TextActor_X);
    m_CGVTKWidget->defaultRenderer()->AddActor(m_TextActor_Y);
    m_CGVTKWidget->defaultRenderer()->AddActor(m_TextActor_Z);
    m_CGVTKWidget->defaultRenderer()->AddActor(m_TextActor_Distance);

    m_CGVTKWidget->update();
}

void CG3DImageView::ShowText3D()
{
    m_CGVTKWidget->update();
}

void CG3DImageView::ShowPCD()
{
    //! qDebug() << "ShowPCD "
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    CG::VTKPointCloudIntensity(g_PointCloud, actor);

    m_Actor = actor;
    m_CGVTKWidget->addActor3D(actor, QColor(25, 50, 75));
    m_CGVTKWidget->defaultRenderer()->ResetCamera();
    m_CGVTKWidget->update();
}

void CG3DImageView::ShowPointPickInfo(const bool enable)
{
    if (enable)
    {
        SetPointPickSize();
        m_CGPointPicker->SetPickEnable(true);
        m_TextActor_X->SetVisibility(1);
        m_TextActor_Y->SetVisibility(1);
        m_TextActor_Z->SetVisibility(1);
        m_CGVTKWidget->defaultRenderer()->AddActor(m_PickSphere_1);
        m_CGVTKWidget->defaultRenderer()->AddActor(m_PickSphere_2);

        if (m_PickType == PickType::SinglePoint) {
            m_CGVTKWidget->defaultRenderer()->RemoveActor(m_PickLine);
            m_TextActor_Distance->SetVisibility(0);
        }
        if (m_PickType == PickType::DoublePoint) {
            m_CGVTKWidget->defaultRenderer()->AddActor(m_PickLine);
            m_TextActor_Distance->SetVisibility(1);
        }
    }
    else
    {
        m_CGPointPicker->SetPickEnable(false);
        m_TextActor_X->SetVisibility(0);
        m_TextActor_Y->SetVisibility(0);
        m_TextActor_Z->SetVisibility(0);
        m_TextActor_Distance->SetVisibility(0);
        m_CGVTKWidget->defaultRenderer()->RemoveActor(m_PickSphere_1);
        m_CGVTKWidget->defaultRenderer()->RemoveActor(m_PickSphere_2);
        m_CGVTKWidget->defaultRenderer()->RemoveActor(m_PickLine);
    }
    m_CGVTKWidget->update();
}

void CG3DImageView::LoadPCD(const std::string filename)
{
    //! qDebug() << "LoadPCD " << QString::fromStdString(filename);
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    CG::LoadPCDFile(filename, actor);

    m_Actor = actor;
    CreatCubeAxes();
    CreatXYGrids(actor->GetBounds());
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
    CreatCubeAxes();
    CreatXYGrids(actor->GetBounds());
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
    CreatCubeAxes();
    CreatXYGrids(actor->GetBounds());
    m_CGVTKWidget->addActor3D(actor, QColor(25, 50, 75));
    m_CGVTKWidget->defaultRenderer()->ResetCamera();
    m_CGVTKWidget->update();
}

void CG3DImageView::LoadPLY(const std::string filename)
{
    //!qDebug() << "LoadPLY " << QString::fromStdString(filename);
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    CG::LoadPLYFile(filename, actor);

    m_Actor = actor;
    CreatCubeAxes();
    CreatXYGrids(actor->GetBounds());
    m_CGVTKWidget->addActor3D(actor, QColor(25, 50, 75));
    m_CGVTKWidget->defaultRenderer()->ResetCamera();
    m_CGVTKWidget->update();
}

void CG3DImageView::LoadSTL(const std::string filename)
{
    //!qDebug() << "LoadSTL " << QString::fromStdString(filename);
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    CG::LoadSTLFile(filename, actor);

    m_Actor = actor;
    CreatCubeAxes();
    CreatXYGrids(actor->GetBounds());
    m_CGVTKWidget->addActor3D(actor, QColor(25, 50, 75));
    m_CGVTKWidget->defaultRenderer()->ResetCamera();
    m_CGVTKWidget->update();
}

void CG3DImageView::LoadOBJ(const std::string filename)
{
    //!qDebug() << "LoadOBJ " << QString::fromStdString(filename);
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    CG::LoadOBJFile(filename, actor);

    m_Actor = actor;
    CreatCubeAxes();
    CreatXYGrids(actor->GetBounds());
    m_CGVTKWidget->addActor3D(actor, QColor(25, 50, 75));
    m_CGVTKWidget->defaultRenderer()->ResetCamera();
    m_CGVTKWidget->update();
}

void CG3DImageView::CreatCubeAxes()
{
    CGVTKUtils::vtkInitOnce(m_CubeAxesActor);
    m_CubeAxesActor->SetCamera(m_CGVTKWidget->defaultRenderer()->GetActiveCamera());
    m_CubeAxesActor->SetBounds(m_Actor->GetBounds());
    m_CubeAxesActor->SetFlyModeToStaticTriad();
    m_CubeAxesActor->SetInertia(1);
    m_CubeAxesActor->SetXAxisTickVisibility(0);
    m_CubeAxesActor->SetYAxisTickVisibility(0);
    m_CubeAxesActor->XAxisMinorTickVisibilityOff();
    m_CubeAxesActor->YAxisMinorTickVisibilityOff();
    m_CubeAxesActor->ZAxisMinorTickVisibilityOff();

    m_CGVTKWidget->defaultRenderer()->AddActor(m_CubeAxesActor);
}

void CG3DImageView::ClearPointCloud()
{
    int num = m_CGVTKWidget->actors3d().count();
    for (int i = 0; i < num; ++i)
    {
        m_CGVTKWidget->defaultRenderer()->RemoveActor(m_CGVTKWidget->actors3d()[i]);
    }
    m_CGVTKWidget->defaultRenderer()->RemoveActor(m_CubeAxesActor);
    m_CGVTKWidget->defaultRenderer()->RemoveActor(m_GridsActor);
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

void CG3DImageView::SetPointPickSize()
{
    double radius = g_XPitch > g_YPitch ? g_XPitch * 5 : g_YPitch * 5;

    if (radius > 0)
    {
        vtkSmartPointer<vtkSphereSource> Sphere_1 = vtkSmartPointer<vtkSphereSource>::New();
        Sphere_1->SetCenter(0, 0, 0);
        Sphere_1->SetRadius(radius);
        Sphere_1->SetThetaResolution(36);
        Sphere_1->SetPhiResolution(36);

        vtkSmartPointer<vtkPolyDataMapper> mapper_1 = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper_1->SetInputConnection(Sphere_1->GetOutputPort());
        m_PickSphere_1->SetMapper(mapper_1);
        m_PickSphere_1->GetProperty()->SetColor(1, 0 ,0);

        vtkSmartPointer<vtkSphereSource> Sphere_2 = vtkSmartPointer<vtkSphereSource>::New();
        Sphere_2->SetCenter(0, 0, 0);
        Sphere_2->SetRadius(radius);
        Sphere_2->SetThetaResolution(36);
        Sphere_2->SetPhiResolution(36);

        vtkSmartPointer<vtkPolyDataMapper> mapper_2 = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper_2->SetInputConnection(Sphere_2->GetOutputPort());
        m_PickSphere_2->SetMapper(mapper_2);
        m_PickSphere_2->GetProperty()->SetColor(1, 0 ,0);
    }
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
    m_CGBoxWidgeter = new CGVTKUtils::CGBoxWidgetObserver();
    m_pBoxWidgetTool->AddObserver(vtkCommand::EndInteractionEvent, m_CGBoxWidgeter);
    connect(m_CGBoxWidgeter, &CGVTKUtils::CGBoxWidgetObserver::planesChanged, this, &CG3DImageView::OnBoxWidgetPlaneChanged);

    m_pBoxWidgetTool->SetInteractor(m_CGVTKWidget->GetInteractor());
    m_pBoxWidgetTool->SetProp3D(m_Actor);
    m_pBoxWidgetTool->GetSelectedFaceProperty()->SetColor(1, 0, 1);
    m_pBoxWidgetTool->GetSelectedFaceProperty()->SetOpacity(0.7);
    m_pBoxWidgetTool->PlaceWidget();
    m_pBoxWidgetTool->On();
}

void CG3DImageView::InitPlaneTool()
{
    m_CGPlaneWidgeter = new CGVTKUtils::CGPlaneWidgetObserver();
    m_pPlaneWidgetTool->AddObserver(vtkCommand::EndInteractionEvent, m_CGPlaneWidgeter);
    connect(m_CGPlaneWidgeter, &CGVTKUtils::CGPlaneWidgetObserver::planesChanged, this, &CG3DImageView::OnPlaneWidgetPlaneChanged);

    m_pPlaneWidgetTool->SetInteractor((m_CGVTKWidget->GetInteractor()));
    m_pPlaneWidgetTool->SetProp3D(m_Actor);
    m_pPlaneWidgetTool->GetPlaneProperty()->SetColor(1, 0, 1);
    m_pPlaneWidgetTool->GetPlaneProperty()->SetOpacity(0.7);
    m_pPlaneWidgetTool->GetPlaneProperty()->SetLineWidth(3);
    m_pPlaneWidgetTool->SetRepresentationToSurface();
    m_pPlaneWidgetTool->NormalToZAxisOn();
    m_pPlaneWidgetTool->PlaceWidget();
    m_pPlaneWidgetTool->On();
}

void CG3DImageView::HandlePickPointCoordinate(float x, float y, float z)
{
    m_PickSphere_1->SetPosition(x, y, z);
    m_PickSphere_2->SetPosition(x, y, z);
}

void CG3DImageView::HandlePickPointDistance(float x, float y, float z)
{
    CG_Point TempPoint;
    TempPoint.x = x;
    TempPoint.y = y;
    TempPoint.z = z;
    s_PickPointsStack.push_back(TempPoint);

    if (s_PickPointsStack.size() == 2)
    {
        m_PickSphere_1->SetPosition(s_PickPointsStack.at(0).x, s_PickPointsStack.at(0).y, s_PickPointsStack.at(0).z);
        m_PickSphere_2->SetPosition(s_PickPointsStack.at(1).x, s_PickPointsStack.at(1).y, s_PickPointsStack.at(1).z);
        m_PickSphere_2->SetVisibility(true);

        m_PickLineSouce->SetPoint1(s_PickPointsStack.at(0).x, s_PickPointsStack.at(0).y, s_PickPointsStack.at(0).z);
        m_PickLineSouce->SetPoint2(s_PickPointsStack.at(1).x, s_PickPointsStack.at(1).y, s_PickPointsStack.at(1).z);
        m_PickLineMapper->SetInputConnection(m_PickLineSouce->GetOutputPort());
        m_PickLine->SetMapper(m_PickLineMapper);
        m_PickLine->SetVisibility(true);

        m_CGVTKWidget->defaultRenderer()->AddActor(m_PickLine);

        float Dist = LineDistance(s_PickPointsStack.at(0).x, s_PickPointsStack.at(0).y, s_PickPointsStack.at(0).z,
                                  s_PickPointsStack.at(1).x, s_PickPointsStack.at(1).y, s_PickPointsStack.at(1).z);
        char chrDist[16];
        sprintf(chrDist, "%.4f", Dist);
        std::string strDist;
        strDist = "D: "; strDist.append(chrDist);
        m_TextActor_Distance->SetInput(strDist.c_str());

        s_PickPointsStack.clear();
    }
    else
    {
        m_PickSphere_1->SetPosition(x, y, z);
        m_PickSphere_2->SetVisibility(false);
        m_PickLine->SetVisibility(false);
    }
}

float CG3DImageView::LineDistance(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float dx = powf((x1 - x2), 2);
    float dy = powf((y1 - y2), 2);
    float dz = powf((z1 - z2), 2);
    float dist = sqrtf(dx + dy + dz);

    return dist;
}

void CG3DImageView::CreatXYGrids(double *bounds)
{
    CGVTKUtils::vtkInitOnce(m_GridsActor);

    int markerCount = 10;
    double xMin = bounds[0];
    double xMax = bounds[1];
    double yMin = bounds[2];
    double yMax = bounds[3];
    double zMin = bounds[4];
    double zMax = bounds[5];

    vtkSmartPointer<vtkAppendPolyData> gridLinesPolyData = vtkSmartPointer<vtkAppendPolyData>::New();

    double deltaX = (double)(xMax - xMin) / (markerCount);
    double initX = xMin;
    while (initX <= xMax)
    {
        vtkSmartPointer<vtkLineSource> ls = vtkSmartPointer<vtkLineSource>::New();
        ls->SetPoint1(initX, yMin, zMin);
        ls->SetPoint2(initX, yMax, zMin);
        ls->Update();
        gridLinesPolyData->AddInputData(ls->GetOutput());
        initX += deltaX;
    }
    if ((initX + deltaX) > xMax)
    {
        vtkSmartPointer<vtkLineSource> ls = vtkSmartPointer<vtkLineSource>::New();
        ls->SetPoint1(xMax, yMin, zMin);
        ls->SetPoint2(xMax, yMax, zMin);
        ls->Update();
        gridLinesPolyData->AddInputData(ls->GetOutput());
    }

    double deltaY = (double)(yMax - yMin) / (markerCount);
    double initY = yMin;
    while (initY <= yMax)
    {
        vtkSmartPointer<vtkLineSource> ls = vtkSmartPointer<vtkLineSource>::New();
        ls->SetPoint1(xMin, initY, zMin);
        ls->SetPoint2(xMax, initY, zMin);
        ls->Update();
        gridLinesPolyData->AddInputData(ls->GetOutput());
        initY += deltaY;
    }
    if ((initY + deltaY) > yMax)
    {
        vtkSmartPointer<vtkLineSource> ls = vtkSmartPointer<vtkLineSource>::New();
        ls->SetPoint1(xMin, yMax, zMin);
        ls->SetPoint2(xMax, yMax, zMin);
        ls->Update();
        gridLinesPolyData->AddInputData(ls->GetOutput());
    }

    gridLinesPolyData->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(gridLinesPolyData->GetOutputPort());

    m_GridsActor->SetMapper(mapper);
    m_CGVTKWidget->defaultRenderer()->AddActor(m_GridsActor);
}

vtkActor* CG3DImageView::GetActor() const
{
    return m_CGVTKWidget->actors3d().back();
}

void CG3DImageView::SetCamera(vtkCamera *camera)
{
    m_CGVTKCamera = camera;
}

void CG3DImageView::SetActor(vtkActor *actor)
{
    m_Actor = actor;
}

void CG3DImageView::InitActors()
{
    CGVTKUtils::vtkInitOnce(m_CGVTKCamera);
    CGVTKUtils::vtkInitOnce(m_Actor);
    CGVTKUtils::vtkInitOnce(m_TextActor_X);
    CGVTKUtils::vtkInitOnce(m_TextActor_Y);
    CGVTKUtils::vtkInitOnce(m_TextActor_Z);
    CGVTKUtils::vtkInitOnce(m_TextActor_Distance);

    CGVTKUtils::vtkInitOnce(m_PickSphere_1);
    CGVTKUtils::vtkInitOnce(m_PickSphere_2);
    CGVTKUtils::vtkInitOnce(m_PickLineSouce);
    CGVTKUtils::vtkInitOnce(m_PickLineMapper);
    CGVTKUtils::vtkInitOnce(m_PickLine);
}

void CG3DImageView::InitTools()
{
    CGVTKUtils::vtkInitOnce(m_pDistanceWidgetTool);
    CGVTKUtils::vtkInitOnce(m_pDistanceRep);
    CGVTKUtils::vtkInitOnce(m_pAngleWidgetTool);
    CGVTKUtils::vtkInitOnce(m_pAngleRep);
    CGVTKUtils::vtkInitOnce(m_pBoxWidgetTool);
    CGVTKUtils::vtkInitOnce(m_pPlaneWidgetTool);

    m_pDistanceRep->GetLineProperty()->SetColor(1, 0, 0);
    m_pDistanceRep->GetLineProperty()->SetLineWidth(3);
    m_pDistanceRep->SetLabelFormat("%.3f");

    m_pAngleRep->GetRay1()->GetProperty()->SetLineWidth(3);
    m_pAngleRep->GetRay2()->GetProperty()->SetLineWidth(3);
    m_pAngleRep->GetArc()->GetProperty()->SetLineWidth(3);
    m_pAngleRep->GetTextActor()->GetProperty()->SetColor(1, 1, 1);
    m_pAngleRep->SetLabelFormat("%.3f");

    m_pBoxWidgetTool->SetPlaceFactor(1.0);
    m_pBoxWidgetTool->SetRotationEnabled(0);
}

void CG3DImageView::InitPointPick()
{
    vtkSmartPointer<vtkPointPicker> PointPicker = vtkSmartPointer<vtkPointPicker>::New();
    m_CGVTKWidget->GetInteractor()->SetPicker(PointPicker);

    m_CGPointPicker = new CGVTKUtils::CGPointPickObserver();
    m_CGPointPicker->SetPickEnable(false);
    m_CGVTKWidget->GetInteractor()->AddObserver(vtkCommand::LeftButtonPressEvent, m_CGPointPicker);

    vtkSmartPointer<vtkSphereSource> Sphere_1 = vtkSmartPointer<vtkSphereSource>::New();
    Sphere_1->SetCenter(0, 0, 0);
    Sphere_1->SetRadius(0.1);
    Sphere_1->SetThetaResolution(36);
    Sphere_1->SetPhiResolution(36);

    vtkSmartPointer<vtkPolyDataMapper> mapper_1 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_1->SetInputConnection(Sphere_1->GetOutputPort());
    m_PickSphere_1->SetMapper(mapper_1);
    m_PickSphere_1->GetProperty()->SetColor(1, 0 ,0);

    vtkSmartPointer<vtkSphereSource> Sphere_2 = vtkSmartPointer<vtkSphereSource>::New();
    Sphere_2->SetCenter(0, 0, 0);
    Sphere_2->SetRadius(0.1);
    Sphere_2->SetThetaResolution(36);
    Sphere_2->SetPhiResolution(36);

    vtkSmartPointer<vtkPolyDataMapper> mapper_2 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_2->SetInputConnection(Sphere_2->GetOutputPort());
    m_PickSphere_2->SetMapper(mapper_2);
    m_PickSphere_2->GetProperty()->SetColor(1, 0 ,0);

    m_PickLine->GetProperty()->SetColor(1, 0, 0);
    m_PickLine->GetProperty()->SetLineWidth(3);
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
            m_CGVTKWidget->GetInteractor()->RemoveObserver(m_CGBoxWidgeter);
            break;

        case PlaneTool:
            m_pPlaneWidgetTool->Off();
            m_CGVTKWidget->GetInteractor()->RemoveObserver(m_CGPlaneWidgeter);
            break;

        default:
            break;
        }
        m_CGVTKWidget->update();

        IsTool = false;
    }
}
