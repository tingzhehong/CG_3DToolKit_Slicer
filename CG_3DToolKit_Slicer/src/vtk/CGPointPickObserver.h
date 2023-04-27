#ifndef CGPOINTPICKOBSERVER_H
#define CGPOINTPICKOBSERVER_H

#include <QObject>
#include <QDebug>
#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>

namespace CGVTKUtils
{

class CGPointPickObserver : public QObject, public vtkCommand
{
    Q_OBJECT

public:
    explicit CGPointPickObserver(QObject *parent = 0);
    ~CGPointPickObserver();

signals:
    void SignalPoint(float x, float y, float z);

public:
    void Execute(vtkObject *caller, unsigned long eventid, void*) override;
    void SetPickEnable(bool enable);
    bool GetPickEnable() const;

private:
    void SinglePick(vtkRenderWindowInteractor *iren, float &x, float &y, float &z);

private:
   float _X, _Y, _Z;
   bool _Enable;
};

}

#endif // CGPOINTPICKOBSERVER_H
