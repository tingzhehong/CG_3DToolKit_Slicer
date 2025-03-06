#ifndef CGPOINTCOORDINATESOBSERVER_H
#define CGPOINTCOORDINATESOBSERVER_H

#include <QObject>
#include <QDebug>
#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>

namespace CGVTKUtils
{

class CGPointCoordinatesObserver : public QObject, public vtkCommand
{
    Q_OBJECT

public:
    explicit CGPointCoordinatesObserver(QObject *parent = 0);
    ~CGPointCoordinatesObserver() = default;

signals:
    void SignalPoint(float x, float y, float z);

public:
    void Execute(vtkObject *caller, unsigned long eventid, void*) override;

private:
    void SinglePick(vtkRenderWindowInteractor *iren, float &x, float &y, float &z);

private:
   float _X, _Y, _Z;
};

}

#endif // CGPOINTCOORDINATESOBSERVER_H
