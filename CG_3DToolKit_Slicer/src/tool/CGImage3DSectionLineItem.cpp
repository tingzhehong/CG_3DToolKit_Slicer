#include "CGImage3DSectionLineItem.h"
#include <CGVTKHeader.h>
#include <CGVTKUtils.h>
#include <CGVTKWidget.h>
#include <vtkSphereSource.h>
#include <vtkPlaneSource.h>
#include <CGOCVHeader.h>

CGImage3DSectionLineItem::CGImage3DSectionLineItem(QObject *parent) : QObject(parent)
{
    InitActor();
    InitConnections();
}

void CGImage3DSectionLineItem::InitActor()
{
    m_Style = vtkSmartPointer<MoveActorInteractorStyle>::New();
    m_Actor = vtkSmartPointer<vtkActor>::New();
    m_SphereActor_1 = vtkSmartPointer<vtkActor>::New();
    m_SphereActor_2 = vtkSmartPointer<vtkActor>::New();
    m_SphereActor_3 = vtkSmartPointer<vtkActor>::New();
    m_Plane = vtkSmartPointer<vtkActor>::New();
}

void CGImage3DSectionLineItem::InitConnections()
{
    connect(m_Style, &MoveActorInteractorStyle::planeMoved, this, &CGImage3DSectionLineItem::OnPositionChange);
}

void CGImage3DSectionLineItem::InitSectionItem()
{
    RemoveSectionItem();
    GetBounds(m_Actor);

    double radius = g_XPitch > g_YPitch ? g_XPitch * 20 : g_YPitch * 20;

    vtkSmartPointer<vtkSphereSource> sphere_1 = vtkSmartPointer<vtkSphereSource>::New();
    sphere_1->SetRadius(radius);
    sphere_1->SetThetaResolution(12);
    sphere_1->SetPhiResolution(12);
    sphere_1->Update();

    vtkSmartPointer<vtkSphereSource> sphere_2 = vtkSmartPointer<vtkSphereSource>::New();
    sphere_2->SetRadius(radius);
    sphere_2->SetThetaResolution(12);
    sphere_2->SetPhiResolution(12);
    sphere_2->Update();

    vtkSmartPointer<vtkSphereSource> sphere_3 = vtkSmartPointer<vtkSphereSource>::New();
    sphere_3->SetRadius(radius);
    sphere_3->SetThetaResolution(12);
    sphere_3->SetPhiResolution(12);
    sphere_3->Update();

    vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
    plane->SetOrigin(m_Bounds[0] + (m_Bounds[1] - m_Bounds[0]) / 2, m_Bounds[2], m_Bounds[4]);
    plane->SetPoint1(m_Bounds[0] + (m_Bounds[1] - m_Bounds[0]) / 2, m_Bounds[2], m_Bounds[5]);
    plane->SetPoint2(m_Bounds[0] + (m_Bounds[1] - m_Bounds[0]) / 2, m_Bounds[3], m_Bounds[4]);

    vtkSmartPointer<vtkPolyDataMapper> mapper_1= vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_1->SetInputConnection(sphere_1->GetOutputPort());

    vtkSmartPointer<vtkPolyDataMapper> mapper_2 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_2->SetInputConnection(sphere_2->GetOutputPort());

    vtkSmartPointer<vtkPolyDataMapper> mapper_3 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_3->SetInputConnection(sphere_3->GetOutputPort());

    vtkSmartPointer<vtkPolyDataMapper> mapper_4 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_4->SetInputConnection(plane->GetOutputPort());

    m_SphereActor_1->SetMapper(mapper_1);
    m_SphereActor_1->GetProperty()->SetColor(1, 0, 0);
    m_SphereActor_1->SetPosition(m_Bounds[0] + (m_Bounds[1] - m_Bounds[0]) / 2, m_Bounds[2], m_Bounds[5]);

    m_SphereActor_2->SetMapper(mapper_2);
    m_SphereActor_2->GetProperty()->SetColor(1, 1, 0);
    m_SphereActor_2->SetPosition(m_Bounds[0] + (m_Bounds[1] - m_Bounds[0]) / 2, m_Bounds[3], m_Bounds[5]);

    m_SphereActor_3->SetMapper(mapper_3);
    m_SphereActor_3->GetProperty()->SetColor(0, 1, 1);
    m_SphereActor_3->SetPosition(m_Bounds[0] + (m_Bounds[1] - m_Bounds[0]) / 2, m_Bounds[2] + (m_Bounds[3] - m_Bounds[2]) / 2, m_Bounds[5]);

    m_Plane->SetMapper(mapper_4);
    m_Plane->GetProperty()->SetColor(1, 0, 1);
    m_Plane->GetProperty()->SetOpacity(0.7);
    m_Plane->SetPickable(false);

    m_CGVTKWidget->defaultRenderer()->AddActor(m_SphereActor_1);
    m_CGVTKWidget->defaultRenderer()->AddActor(m_SphereActor_2);
    m_CGVTKWidget->defaultRenderer()->AddActor(m_SphereActor_3);
    m_CGVTKWidget->defaultRenderer()->AddActor(m_Plane);
    m_CGVTKWidget->update();

    m_Style->bounds = m_Bounds;
    m_Style->m_sphereActor_1 = m_SphereActor_1;
    m_Style->m_sphereActor_2 = m_SphereActor_2;
    m_Style->m_sphereActor_3 = m_SphereActor_3;
    m_Style->m_plane = m_Plane;

    m_CGVTKWidget->update();
}

void CGImage3DSectionLineItem::SetInteractorStyleMoveActor()
{
    m_Style->SetDefaultRenderer(m_CGVTKWidget->defaultRenderer());
    m_CGVTKWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(m_Style);
    m_CGVTKWidget->update();
}

void CGImage3DSectionLineItem::SetInteractorStyleDefault()
{
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    m_Style->SetDefaultRenderer(m_CGVTKWidget->defaultRenderer());
    m_CGVTKWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(style);
    m_CGVTKWidget->update();
}

void CGImage3DSectionLineItem::RemoveSectionItem()
{
    SetInteractorStyleDefault();
    m_CGVTKWidget->defaultRenderer()->RemoveActor(m_SphereActor_1);
    m_CGVTKWidget->defaultRenderer()->RemoveActor(m_SphereActor_2);
    m_CGVTKWidget->defaultRenderer()->RemoveActor(m_SphereActor_3);
    m_CGVTKWidget->defaultRenderer()->RemoveActor(m_Plane);
    m_CGVTKWidget->update();
}

void CGImage3DSectionLineItem::OnPositionChange(double *pos_1, double *pos_2)
{
    m_CGVTKWidget->update();

    if (m_Bounds[0] == m_Bounds[1]) return;
    if (m_Bounds[2] == m_Bounds[3]) return;
    
    double posact_1[2];
    posact_1[0] = (pos_1[0] - m_Bounds[0]) / (m_Bounds[1] - m_Bounds[0]);
    posact_1[1] = (pos_1[1] - m_Bounds[2]) / (m_Bounds[3] - m_Bounds[2]);

    double posact_2[2];
    posact_2[0] = (pos_2[0] - m_Bounds[0]) / (m_Bounds[1] - m_Bounds[0]);
    posact_2[1] = (pos_2[1] - m_Bounds[2]) / (m_Bounds[3] - m_Bounds[2]);

    emit SignalPositionChange(posact_1, posact_2);
}

void CGImage3DSectionLineItem::OnUpdate()
{
    m_CGVTKWidget->update();
}

void CGImage3DSectionLineItem::SetVTKWidget(CGVTKWidget *widget)
{
    m_CGVTKWidget = widget;
}

void CGImage3DSectionLineItem::SetActor(vtkSmartPointer<vtkActor> actor)
{
    m_Actor = actor;
    m_Actor->SetPickable(0);
}

void CGImage3DSectionLineItem::GetBounds(vtkSmartPointer<vtkActor> actor)
{
    actor->GetBounds(m_Bounds);
}
