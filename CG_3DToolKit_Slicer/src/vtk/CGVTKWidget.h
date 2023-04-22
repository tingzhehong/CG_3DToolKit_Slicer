#ifndef CGVTKWIDGET_H
#define CGVTKWIDGET_H

#include <QVTKWidget.h>
#include <vtkAutoInit.h>

VTK_MODULE_INIT(vtkRenderingOpenGL)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingFreeType)

class vtkActor;
class vtkProp;
class CGVTKWidgetPrivate;
class CGVTKWidget : public QVTKWidget
{
    Q_OBJECT

public:
    explicit CGVTKWidget(QWidget* parent = nullptr);
    virtual ~CGVTKWidget();

    void setMultiViewports(bool multi = true);
    bool multiViewports() const;

    void addActor(vtkProp* actor, const QColor& clr = Qt::black);
    void addActor3D(vtkActor* actor3d, const QColor& clr = Qt::black);
    void addViewProp(vtkProp* prop);
    QList<vtkProp*> actors() const;
    QList<vtkActor*> actors3d() const;

    void setActorsVisible(bool visible);
    void setActorVisible(vtkProp* actor, bool visible);
    bool actorVisible(vtkProp* actor);

    void setActors3DVisible(bool visible);
    void setActor3DVisible(vtkActor* actor3d, bool visible);
    bool actor3DVisible(vtkActor* actor3d);

    void setBackgroundColor(const QColor& clr);
    QColor backgroundColor() const;

    vtkRenderer* defaultRenderer();
    bool defaultRendererTaken() const;

    void showOrientationMarker(bool show = true);

protected:
    void setBounds(double* bounds);

    double xMin() const;
    double xMax() const;
    double yMin() const;
    double yMax() const;
    double zMin() const;
    double zMax() const;

private:
    CGVTKWidgetPrivate* d_ptr;
    Q_DISABLE_COPY(CGVTKWidget)
};

#endif // CGVTKWIDGET_H
