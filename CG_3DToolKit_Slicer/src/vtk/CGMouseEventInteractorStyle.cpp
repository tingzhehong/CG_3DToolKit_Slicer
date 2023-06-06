#include "CGMouseEventInteractorStyle.h"

#include <vtkObjectFactory.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPointPicker.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <Utils.h>
#include <CGVtkUtils.h>
#include <QDebug>

vtkStandardNewMacro(MouseEventInteractorStyle)


void MouseEventInteractorStyle::OnLeftButtonDown()
{
    this->StartRotate();
}

void MouseEventInteractorStyle::OnLeftButtonUp()
{
    this->EndRotate();
}

void MouseEventInteractorStyle::OnRightButtonDown()
{
    updateCurrentPos();
    getPressedActor();
    emit mousePressed(m_pos);
}

void MouseEventInteractorStyle::OnRightButtonUp()
{
    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
    updateCurrentPos();
    m_pickedActor = nullptr;
    emit mouseReleased(m_pos);
}

void MouseEventInteractorStyle::OnMouseMove()
{
    if (this->State == VTKIS_ROTATE)
        vtkInteractorStyleTrackballCamera::OnMouseMove();

    updateCurrentPos();

    if (m_moveActor && m_pickedActor)
    {
        switch (m_SectionType)
        {
        case SectionItemVertical:
            m_pos[0] = m_pos[0] - (_bounds[0] + (_bounds[1] - _bounds[0]) / 2);
            m_pickedActor->SetPosition(m_pos[0], _pos[1], _pos[2]);
            break;
        case SectionItemHorizontal:
            m_pos[1] = m_pos[1] - (_bounds[2] + (_bounds[3] - _bounds[2]) / 2);
            m_pickedActor->SetPosition(_pos[0], m_pos[1], _pos[2]);
            break;
        default:
            m_pickedActor->SetPosition(m_pos);
            break;
        }
        GetDefaultRenderer()->GetRenderWindow()->Render();
        emit mouseMoved(m_pos);
    }
}

void MouseEventInteractorStyle::OnKeyDown()
{
    vtkInteractorStyleTrackballCamera::OnKeyDown();
    emit ctrlPressed(GetInteractor()->GetControlKey());
}

void MouseEventInteractorStyle::OnKeyUp()
{
    vtkInteractorStyleTrackballCamera::OnKeyUp();
    emit ctrlPressed(GetInteractor()->GetControlKey());
}

void MouseEventInteractorStyle::setMoveActor(bool move)
{
    m_moveActor = move;
}

bool MouseEventInteractorStyle::moveActor() const
{
    return m_moveActor;
}

void MouseEventInteractorStyle::updateCurrentPos()
{
    int x = GetInteractor()->GetEventPosition()[0];
    int y = GetInteractor()->GetEventPosition()[1];

    VTK_CREATE(vtkPointPicker, picker);
    picker->SetTolerance(0.001);
    picker->Pick(x, y, 0, GetDefaultRenderer());
    VTKUtils::ArrayAssigner<double>()(m_pos, picker->GetPickPosition());
}

void MouseEventInteractorStyle::getPressedActor()
{
    int x = GetInteractor()->GetEventPosition()[0];
    int y = GetInteractor()->GetEventPosition()[1];

    VTK_CREATE(vtkPicker, picker);
    picker->SetTolerance(0.001);
    picker->Pick(x, y, 0, GetDefaultRenderer());
    m_pickedActor = picker->GetActor();
    if (m_pickedActor)
    {
        m_pickedActor->GetPosition(_pos);
        m_pickedActor->GetBounds(_bounds);
    }
}
