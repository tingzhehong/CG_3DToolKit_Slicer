#include "CGImage3DSectionItemVertical.h"
#include <CGVTKHeader.h>
#include <CGVTKUtils.h>
#include <CGVTKWidget.h>
#include <vtkSphereSource.h>
#include <vtkPlaneSource.h>

CGImage3DSectionItemVertical::CGImage3DSectionItemVertical(QObject *parent) : QObject(parent)
{
    InitActor();
    InitConnections();
}

void CGImage3DSectionItemVertical::InitActor()
{
    m_Style = vtkSmartPointer<MouseEventInteractorStyle>::New();
    m_Style->setMoveActor(true);
    m_Plane = vtkSmartPointer<vtkActor>::New();
}

void CGImage3DSectionItemVertical::InitConnections()
{
    connect(m_Style, &MouseEventInteractorStyle::mouseMoved, this, &CGImage3DSectionItemVertical::OnPositionChange);
    connect(m_Style, &MouseEventInteractorStyle::mouseReleased, this, &CGImage3DSectionItemVertical::OnUpdate);
}

void CGImage3DSectionItemVertical::InitSectionItem()
{
    RemoveSectionItem();
    GetBounds(m_Actor);

    vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
    plane->SetOrigin(m_Bounds[0] + (m_Bounds[1] - m_Bounds[0]) / 2, m_Bounds[2], m_Bounds[4]);
    plane->SetPoint1(m_Bounds[0] + (m_Bounds[1] - m_Bounds[0]) / 2, m_Bounds[2], m_Bounds[5]);
    plane->SetPoint2(m_Bounds[0] + (m_Bounds[1] - m_Bounds[0]) / 2, m_Bounds[3], m_Bounds[4]);

    vtkSmartPointer<vtkPolyDataMapper> mapper_plane  = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_plane ->SetInputConnection(plane->GetOutputPort());
    m_Plane->SetMapper(mapper_plane);
    m_Plane->GetProperty()->SetColor(1, 0, 1);
    m_Plane->GetPosition(m_PlanePos);

    m_CGVTKWidget->defaultRenderer()->AddActor(m_Plane);
    m_CGVTKWidget->update();
}

void CGImage3DSectionItemVertical::SetInteractorStyleDefault()
{
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    m_Style->SetDefaultRenderer(m_CGVTKWidget->defaultRenderer());
    m_CGVTKWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(style);
    m_CGVTKWidget->update();
}

void CGImage3DSectionItemVertical::SetInteractorStyleMouseEvent()
{
    m_Style->SetDefaultRenderer(m_CGVTKWidget->defaultRenderer());
    m_CGVTKWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(m_Style);
    m_CGVTKWidget->update();
}

void CGImage3DSectionItemVertical::RemoveSectionItem()
{
    SetInteractorStyleDefault();
    m_CGVTKWidget->defaultRenderer()->RemoveActor(m_Plane);
    m_CGVTKWidget->update();
}

void CGImage3DSectionItemVertical::OnPositionChange(double* pos)
{
    m_Plane->SetPosition(pos[0], m_PlanePos[1], m_PlanePos[2]);
    emit SignalPositionChange(pos[0]);
}

void CGImage3DSectionItemVertical::OnUpdate()
{
    m_CGVTKWidget->update();
}

void CGImage3DSectionItemVertical::GetBounds(vtkSmartPointer<vtkActor> actor)
{
    actor->GetBounds(m_Bounds);
}

void CGImage3DSectionItemVertical::SetVTKWidget(CGVTKWidget *widget)
{
    m_CGVTKWidget = widget;
}

void CGImage3DSectionItemVertical::SetActor(vtkSmartPointer<vtkActor>actor)
{
    m_Actor = actor;
    m_Actor->SetPickable(0);
}
