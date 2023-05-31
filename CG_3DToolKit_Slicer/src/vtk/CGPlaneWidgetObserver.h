#ifndef CGPLANEWIDGETOBSERVER_H
#define CGPLANEWIDGETOBSERVER_H

#include "CGAbstractWidgetObserver.h"
#include <vtkSmartPointer.h>

class vtkPlane;
namespace CGVTKUtils
{
class CGPlaneWidgetObserver : public AbstractWidgetObserver
{
    Q_OBJECT

public:
    explicit CGPlaneWidgetObserver();

signals:
    void planesChanged(vtkPlane* plane);

protected:
    void Execute(vtkObject *caller, unsigned long eventId, void* callData);

    vtkSmartPointer<vtkPlane> m_plane;
};

}

#endif // CGPLANEWIDGETOBSERVER_H
