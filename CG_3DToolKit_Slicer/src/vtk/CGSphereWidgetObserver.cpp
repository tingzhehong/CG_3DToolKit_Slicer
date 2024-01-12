#include "CGSphereWidgetObserver.h"

#include <vtkSphereWidget.h>

namespace CGVTKUtils
{

CGSphereWidgetObserver::CGSphereWidgetObserver(QObject* parent) : AbstractWidgetObserver(parent)
{
    p = sphere;
}

void CGSphereWidgetObserver::Execute(vtkObject *caller, unsigned long eventId, void *callData)
{
    Q_UNUSED(eventId)
    Q_UNUSED(callData)

    vtkSphereWidget* widget = reinterpret_cast<vtkSphereWidget*>(caller);
    if (widget) {
        widget->GetCenter(center);
        sphere[0] = center[0];
        sphere[1] = center[1];
        sphere[2] = center[2];

        radius = widget->GetRadius();
        sphere[3] = radius;

        emit sphereChanged(p);
    }
}

}
