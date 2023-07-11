#include "CGBoxWidgetObserver.h"

#include <vtkBoxWidget.h>
#include <vtkPlanes.h>

namespace CGVTKUtils
{

CGBoxWidgetObserver::CGBoxWidgetObserver(QObject* parent) : AbstractWidgetObserver(parent)
{
    m_planes = vtkPlanes::New();
}

void CGBoxWidgetObserver::Execute(vtkObject *caller, unsigned long eventId, void* callData)
{
    Q_UNUSED(eventId)
    Q_UNUSED(callData)

    vtkBoxWidget* widget = reinterpret_cast<vtkBoxWidget*>(caller);
    if (widget) {
        widget->GetPlanes(m_planes);
        emit planesChanged(m_planes);
    }

}

} // namespace CGVTKUtils
