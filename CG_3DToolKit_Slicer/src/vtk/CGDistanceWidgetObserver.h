#ifndef DISTANCEWIDGETOBSERVER_H
#define DISTANCEWIDGETOBSERVER_H

#include "CGAbstractWidgetObserver.h"

namespace CGVTKUtils
{

class CGDistanceWidgetObserver : public AbstractWidgetObserver
{
    Q_OBJECT
    
public:
    explicit CGDistanceWidgetObserver(QObject* parent = nullptr);

signals:
    void distanceChanged(double dist);
    void worldPoint1Changed(double* pos);
    void worldPoint2Changed(double* pos);
    void displayPoint1Changed(double* pos);
    void displayPoint2Changed(double* pos);

protected:
    void Execute(vtkObject *caller, unsigned long eventId, void* callData);
};

} // namespace CGVTKUtils
#endif // DISTANCEWIDGETOBSERVER_H
