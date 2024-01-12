#ifndef CGSPHEREWIDGETOBSERVER_H
#define CGSPHEREWIDGETOBSERVER_H

#include "CGAbstractWidgetObserver.h"
#include <vtkSmartPointer.h>

class vtkSphereWidget;
namespace CGVTKUtils
{

class CGSphereWidgetObserver : public AbstractWidgetObserver
{
    Q_OBJECT

public:
    explicit CGSphereWidgetObserver(QObject* parent = nullptr);

signals:
    void sphereChanged(double* sphere);

protected:
    void Execute(vtkObject *caller, unsigned long eventId, void* callData);

    double sphere[4];
    double center[3];
    double radius;
    double *p;
};

}

#endif // CGSPHEREWIDGETOBSERVER_H
