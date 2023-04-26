#ifndef CGPOINTPICKOBSERVER_H
#define CGPOINTPICKOBSERVER_H

#include <QObject>
#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>

namespace CGVTKUtils
{

class CGPointPickObserver: public QObject, public vtkCommand
{
    Q_OBJECT

public:
    explicit CGPointPickObserver(QObject *parent = 0);
    ~CGPointPickObserver();

signals:
    void SignalPoint(float &x, float &y, float &z);

public:
    void Execute(vtkObject *caller, unsigned long eventid, void*) override;

private:
    void SinglePick(vtkRenderWindowInteractor *iren, float &x, float &y, float &z);

private:
   float X, Y, Z;

};

}

#endif // CGPOINTPICKOBSERVER_H
