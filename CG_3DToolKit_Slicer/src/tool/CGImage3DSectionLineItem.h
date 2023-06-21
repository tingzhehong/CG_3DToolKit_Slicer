#ifndef CGIMAGE3DSECTIONLINEITEM_H
#define CGIMAGE3DSECTIONLINEITEM_H

#include <QObject>
#include <vtkSmartPointer.h>
#include <CGMoveActorInteractorStyle.h>

class vtkActor;
class CGVTKWidget;
class CGImage3DSectionLineItem : public QObject
{
    Q_OBJECT

public:
    explicit CGImage3DSectionLineItem(QObject *parent = nullptr);
    ~CGImage3DSectionLineItem() = default;

    void InitActor();
    void InitConnections();
    void InitSectionItem();
    void SetInteractorStyleMoveActor();
    void SetInteractorStyleDefault();
    void RemoveSectionItem();

signals:
    void SignalPositionChange(double pos_1[], double pos_2[]);

private slots:
    void OnPositionChange(double* pos_1, double* pos_2);
    void OnUpdate();

public:
    void SetVTKWidget(CGVTKWidget* widget);
    void SetActor(vtkSmartPointer<vtkActor> actor);
    void GetBounds(vtkSmartPointer<vtkActor> actor);

public:
    CGVTKWidget *m_CGVTKWidget = nullptr;

    vtkSmartPointer<MoveActorInteractorStyle> m_Style;
    vtkSmartPointer<vtkActor> m_Actor;
    vtkSmartPointer<vtkActor> m_SphereActor_1;
    vtkSmartPointer<vtkActor> m_SphereActor_2;
    vtkSmartPointer<vtkActor> m_SphereActor_3;
    vtkSmartPointer<vtkActor> m_Plane;

private:
    double m_Bounds[6];
};

#endif // CGIMAGE3DSECTIONLINEITEM_H
