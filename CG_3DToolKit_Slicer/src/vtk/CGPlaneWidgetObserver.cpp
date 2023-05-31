#include "CGPlaneWidgetObserver.h"

#include <vtkPlaneWidget.h>
#include <vtkPlane.h>

namespace CGVTKUtils
{
CGPlaneWidgetObserver::CGPlaneWidgetObserver()
{
    m_plane = vtkPlane::New();
}

void CGPlaneWidgetObserver::Execute(vtkObject *caller, unsigned long eventId, void *callData)
{
    Q_UNUSED(eventId)
    Q_UNUSED(callData)

    vtkPlaneWidget* widget = reinterpret_cast<vtkPlaneWidget*>(caller);
    if (widget) {
        widget->GetPlane(m_plane);
        emit planesChanged(m_plane);
    }
}

}
