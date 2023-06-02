#ifndef CGIMAGE3DSECTIONITEMHORIZONTAL_H
#define CGIMAGE3DSECTIONITEMHORIZONTAL_H

#include <QObject>
#include <vtkSmartPointer.h>
#include <CGMouseEventInteractorStyle.h>

class vtkActor;
class CGVTKWidget;
class CGImage3DSectionItemHorizontal : public QObject
{
    Q_OBJECT

public:
    explicit CGImage3DSectionItemHorizontal(QObject *parent = nullptr);
    ~CGImage3DSectionItemHorizontal() = default;

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

#endif // CGIMAGE3DSECTIONITEMHORIZONTAL_H
