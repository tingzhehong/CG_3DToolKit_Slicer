#ifndef ABSTRACTWIDGETOBSERVER_H
#define ABSTRACTWIDGETOBSERVER_H

#include <QObject>
#include <vtkCommand.h>

class vtkInteractorObserver;
namespace CGVTKUtils
{

class AbstractWidgetObserver : public QObject, public vtkCommand
{
    Q_OBJECT
    
public:
    explicit AbstractWidgetObserver(QObject *parent = 0);
    virtual ~AbstractWidgetObserver();

    void attach(vtkInteractorObserver* widget);

protected:
    virtual void Execute(vtkObject *caller, unsigned long eventId, void* callData) = 0;

protected:
    vtkInteractorObserver* m_widget = nullptr;
};

} // namespace CGVTKUtils

#endif // ABSTRACTWIDGETOBSERVER_H
