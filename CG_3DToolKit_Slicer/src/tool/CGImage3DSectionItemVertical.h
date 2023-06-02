#ifndef CGIMAGE3DSECTIONITEMVERTICAL_H
#define CGIMAGE3DSECTIONITEMVERTICAL_H

#include <QObject>
#include <vtkSmartPointer.h>
#include <CGMouseEventInteractorStyle.h>

class vtkActor;
class CGVTKWidget;
class CGImage3DSectionItemVertical : public QObject
{
    Q_OBJECT

public:
    explicit CGImage3DSectionItemVertical(QObject *parent = nullptr);
    ~CGImage3DSectionItemVertical() = default;

    void InitActor();
    void InitConnections();
    void InitSectionItem();
    void SetInteractorStyleMouseEvent();
    void SetInteractorStyleDefault();
    void RemoveSectionItem();

signals:
    void SignalPositionChange(double pos);

private slots:
    void OnPositionChange(double* pos);
    void OnUpdate();

public:
    void SetVTKWidget(CGVTKWidget* widget);
    void SetActor(vtkSmartPointer<vtkActor> actor);
    void GetBounds(vtkSmartPointer<vtkActor> actor);

public:
    CGVTKWidget *m_CGVTKWidget = nullptr;

    vtkSmartPointer<MouseEventInteractorStyle> m_Style;
    vtkSmartPointer<vtkActor> m_Actor;
    vtkSmartPointer<vtkActor> m_Plane;

private:
    double m_Bounds[6];
    double m_PlanePos[3];
};

#endif // CGIMAGE3DSECTIONITEMVERTICAL_H
