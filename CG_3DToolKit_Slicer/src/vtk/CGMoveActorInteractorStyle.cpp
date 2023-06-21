#include "CGMoveactorInteractorStyle.h"

#include <vtkObjectFactory.h>
#include <vtkPickingManager.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPlaneSource.h>
#include <vtkPicker.h>
#include <vtkCellPicker.h>
#include <vtkPropPicker.h>
#include <vtkPointPicker.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkExtractEdges.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <Utils.h>
#include <CGVtkUtils.h>
#include <QDebug>


vtkStandardNewMacro(MoveActorInteractorStyle)

void MoveActorInteractorStyle::OnLeftButtonDown()
{
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();

    int x = GetInteractor()->GetEventPosition()[0];
    int y = GetInteractor()->GetEventPosition()[1];

    vtkSmartPointer<vtkPicker> picker = vtkSmartPointer<vtkPicker>::New();
    picker->SetTolerance(0.001);
    vtkRenderer* defaultRenderer = GetDefaultRenderer();
    picker->Pick(x, y, 0, defaultRenderer);
    m_pickedActor = picker->GetActor();

	if (m_pickedActor)
	{
		lastPos_1 = m_sphereActor_1->GetPosition();
		lastPos_2 = m_sphereActor_2->GetPosition();
		lastPos_3 = m_sphereActor_3->GetPosition();
	}
}

void MoveActorInteractorStyle::OnLeftButtonUp()
{
    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();

    m_pickedActor = nullptr;
}

void MoveActorInteractorStyle::OnMouseMove()
{
    vtkInteractorStyleTrackballCamera::OnMouseMove();

    if (m_pickedActor)
	{
        int x = GetInteractor()->GetEventPosition()[0];
        int y = GetInteractor()->GetEventPosition()[1];

		vtkSmartPointer<vtkPicker> picker = vtkSmartPointer<vtkPicker>::New();
        picker->SetTolerance(0.001);
        vtkRenderer* defaultRenderer = GetDefaultRenderer();
        picker->Pick(x, y, 0, defaultRenderer);

        double* movePos = picker->GetPickPosition();
        /*m_pickedActor->SetPosition(movePos);*/
		m_pickedActor->SetPosition(movePos[0], movePos[1], bounds[5]);
        defaultRenderer->GetRenderWindow()->Render();

		OnPlaneMove();
		defaultRenderer->GetRenderWindow()->Render();

        emit planeMoved(m_sphereActor_1->GetPosition(), m_sphereActor_2->GetPosition());
    }
}

void MoveActorInteractorStyle::Rotate()
{
	if (m_pickedActor)
		return;

	vtkInteractorStyleTrackballCamera::Rotate();
}

void MoveActorInteractorStyle::Spin()
{
	if (m_pickedActor)
		return;

	vtkInteractorStyleTrackballCamera::Spin();
}

void MoveActorInteractorStyle::OnChar()
{
    vtkInteractorStyleTrackballCamera::OnChar();
}

void MoveActorInteractorStyle::OnPlaneMove()
{
	double *spherePosition_1 = m_sphereActor_1->GetPosition();
	double *spherePosition_2 = m_sphereActor_2->GetPosition();
	double *spherePosition_3 = m_sphereActor_3->GetPosition();

	double d[3];
	d[0] = spherePosition_1[0] > spherePosition_2[0] ? (spherePosition_1[0] - spherePosition_2[0]) / 2 + spherePosition_2[0] : (spherePosition_2[0] - spherePosition_1[0]) / 2 + spherePosition_1[0];
	d[1] = spherePosition_1[1] > spherePosition_2[1] ? (spherePosition_1[1] - spherePosition_2[1]) / 2 + spherePosition_2[1] : (spherePosition_2[1] - spherePosition_1[1]) / 2 + spherePosition_1[1];
	d[2] = spherePosition_3[2];

	if (m_pickedActor == m_sphereActor_1 || m_pickedActor == m_sphereActor_2)
	{
		m_sphereActor_3->SetPosition(d);
	}

	if (m_pickedActor == m_sphereActor_3)
	{
		double deltaX = spherePosition_3[0] - d[0];
		double deltaY = spherePosition_3[1] - d[1];
		
		m_sphereActor_1->SetPosition(lastPos_1[0] + deltaX, lastPos_1[1] + deltaY, spherePosition_1[2]);
		m_sphereActor_2->SetPosition(lastPos_2[0] + deltaX, lastPos_2[1] + deltaY, spherePosition_2[2]);
	}

	vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
	plane->SetOrigin(spherePosition_1[0], spherePosition_1[1], bounds[4]);
	plane->SetPoint1(spherePosition_1[0], spherePosition_1[1], bounds[5]);
	plane->SetPoint2(spherePosition_2[0], spherePosition_2[1], bounds[4]);

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(plane->GetOutputPort());
    m_plane->SetMapper(mapper);
}

MoveActorInteractorStyle::MoveActorInteractorStyle(QObject *parent) : QObject(parent)
{

}
