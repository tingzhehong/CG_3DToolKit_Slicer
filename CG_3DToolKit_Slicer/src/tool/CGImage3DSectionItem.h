#ifndef CGIMAGE3DSECTIONITEM_H
#define CGIMAGE3DSECTIONITEM_H

#include <QObject>
#include <vtkSmartPointer.h>
#include <CGMouseEventInteractorStyle.h>

class vtkActor;
class CGVTKWidget;
class CGImage3DSectionItem : public QObject
{
    Q_OBJECT

public:
    explicit CGImage3DSectionItem(QObject *parent = nullptr);
    ~CGImage3DSectionItem() = default;

    void InitActor();
    void InitConnections();
    void InitSectionItem();
    void SetInteractorStyle();

private slots:
    void OnPositionChange(double* pos);

public:
    void SetVTKWidget(CGVTKWidget* widget);
    void SetActor(vtkSmartPointer<vtkActor> actor);
    void GetBounds(vtkSmartPointer<vtkActor> actor);

private:
    CGVTKWidget *m_CGVTKWidget = nullptr;

    vtkSmartPointer<MouseEventInteractorStyle> m_Style;
    vtkSmartPointer<vtkActor> m_Actor;
    vtkSmartPointer<vtkActor> m_Sphere1;
    vtkSmartPointer<vtkActor> m_Sphere2;
    vtkSmartPointer<vtkActor> m_Plane;

private:
    double m_Bounds[6];
    double m_Sphere1Pos[3];
    double m_Sphere2Pos[3];

};

#endif // CGIMAGE3DSECTIONITEM_H
