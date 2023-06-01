#include "CGImage3DSectionItem.h"
#include <CGVTKHeader.h>
#include <CGVTKUtils.h>
#include <CGVTKWidget.h>
#include <vtkSphereSource.h>
#include <vtkPlaneSource.h>

CGImage3DSectionItem::CGImage3DSectionItem(QObject *parent) : QObject(parent)
{
    InitActor();
    InitConnections();
}

void CGImage3DSectionItem::InitActor()
{
    m_Style = vtkSmartPointer<MouseEventInteractorStyle>::New();
    m_Style->setMoveActor(true);
    m_Sphere1 = vtkSmartPointer<vtkActor>::New();
    m_Sphere2 = vtkSmartPointer<vtkActor>::New();
    m_Plane = vtkSmartPointer<vtkActor>::New();
}

void CGImage3DSectionItem::InitConnections()
{
    connect(m_Style, &MouseEventInteractorStyle::mouseReleased, this, &CGImage3DSectionItem::OnPositionChange);
}

void CGImage3DSectionItem::InitSectionItem()
{
    GetBounds(m_Actor);

    m_Sphere1Pos[0] = m_Bounds[0] + (m_Bounds[1] - m_Bounds[0]) / 2;
    m_Sphere1Pos[1] = m_Bounds[2];
    m_Sphere1Pos[2] = m_Bounds[5];

    m_Sphere2Pos[0] = m_Bounds[0] + (m_Bounds[1] - m_Bounds[0]) / 2;
    m_Sphere2Pos[1] = m_Bounds[3];
    m_Sphere2Pos[2] = m_Bounds[5];

    vtkSmartPointer<vtkSphereSource> Sphere_1 = vtkSmartPointer<vtkSphereSource>::New();
    Sphere_1->SetCenter(m_Sphere1Pos[0], m_Sphere1Pos[1], m_Sphere1Pos[2]);
    Sphere_1->SetRadius(0.01);
    Sphere_1->SetThetaResolution(36);
    Sphere_1->SetPhiResolution(36);

    vtkSmartPointer<vtkPolyDataMapper> mapper_1 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_1->SetInputConnection(Sphere_1->GetOutputPort());
    m_Sphere1->SetMapper(mapper_1);
    m_Sphere1->GetProperty()->SetColor(0, 0, 1);


    vtkSmartPointer<vtkSphereSource> Sphere_2 = vtkSmartPointer<vtkSphereSource>::New();
    Sphere_2->SetCenter(m_Sphere2Pos[0], m_Sphere2Pos[1], m_Sphere2Pos[2]);
    Sphere_2->SetRadius(0.01);
    Sphere_2->SetThetaResolution(36);
    Sphere_2->SetPhiResolution(36);

    vtkSmartPointer<vtkPolyDataMapper> mapper_2 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_2->SetInputConnection(Sphere_2->GetOutputPort());
    m_Sphere2->SetMapper(mapper_2);
    m_Sphere2->GetProperty()->SetColor(1, 1, 0);


    vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
    plane->SetOrigin(m_Sphere1Pos[0], m_Sphere1Pos[1], m_Bounds[4]);
    plane->SetPoint1(m_Sphere1Pos[0], m_Sphere1Pos[1], m_Bounds[5]);
    plane->SetPoint2(m_Sphere2Pos[0], m_Sphere2Pos[1], m_Bounds[4]);

    vtkSmartPointer<vtkPolyDataMapper> mapper_3 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_3->SetInputConnection(plane->GetOutputPort());
    m_Plane->SetMapper(mapper_3);
    m_Plane->GetProperty()->SetColor(1, 0, 1);

    m_CGVTKWidget->defaultRenderer()->AddActor(m_Sphere1);
    m_CGVTKWidget->defaultRenderer()->AddActor(m_Sphere2);
    m_CGVTKWidget->defaultRenderer()->AddActor(m_Plane);
    m_CGVTKWidget->update();
}

void CGImage3DSectionItem::SetInteractorStyle()
{
    m_Style->SetDefaultRenderer(m_CGVTKWidget->defaultRenderer());
    m_CGVTKWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(m_Style);
    m_CGVTKWidget->update();
}

void CGImage3DSectionItem::OnPositionChange(double* pos)
{
    if (pos[0] != m_Sphere1Pos[0] || pos[1] != m_Sphere1Pos[1])
    {
        vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
        plane->SetOrigin(m_Sphere1Pos[0], m_Sphere1Pos[1], m_Bounds[4]);
        plane->SetPoint1(m_Sphere1Pos[0], m_Sphere1Pos[1], m_Bounds[5]);
        plane->SetPoint2(m_Sphere2Pos[0], m_Sphere2Pos[1], m_Bounds[5]);

        vtkSmartPointer<vtkPolyDataMapper> mapper_3 = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper_3->SetInputConnection(plane->GetOutputPort());
        m_Plane->SetMapper(mapper_3);

        m_Sphere1Pos[0] = pos[0];
        m_Sphere1Pos[1] = pos[1];
        m_Sphere1->SetPosition(pos[0], pos[1], m_Sphere1Pos[2]);
    }
    else if (pos[0] != m_Sphere2Pos[0] || pos[1] != m_Sphere2Pos[1])
    {
        vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
        plane->SetOrigin(m_Sphere1Pos[0], m_Sphere1Pos[1], m_Bounds[4]);
        plane->SetPoint1(m_Sphere1Pos[0], m_Sphere1Pos[1], m_Bounds[5]);
        plane->SetPoint2(m_Sphere2Pos[0], m_Sphere2Pos[1], m_Bounds[5]);

        vtkSmartPointer<vtkPolyDataMapper> mapper_3 = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper_3->SetInputConnection(plane->GetOutputPort());
        m_Plane->SetMapper(mapper_3);

        m_Sphere2Pos[0] = pos[0];
        m_Sphere2Pos[1] = pos[1];
        m_Sphere2->SetPosition(pos[0], pos[1], m_Sphere2Pos[2]);
    }

    m_CGVTKWidget->update();
}

void CGImage3DSectionItem::GetBounds(vtkSmartPointer<vtkActor> actor)
{
    actor->GetBounds(m_Bounds);
}

void CGImage3DSectionItem::SetVTKWidget(CGVTKWidget *widget)
{
    m_CGVTKWidget = widget;
}

void CGImage3DSectionItem::SetActor(vtkSmartPointer<vtkActor>actor)
{
    m_Actor = actor;
}
