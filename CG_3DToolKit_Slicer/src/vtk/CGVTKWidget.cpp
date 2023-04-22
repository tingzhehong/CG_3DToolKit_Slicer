#include "CGVTKWidget.h"
#include <vtkRenderWindow.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindow.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>

#include <Utils.h>
#include <CGRenderersLayoutAlg.h>
#include <CGVTKUtils.h>

class CGVTKWidgetPrivate
{
public:
    CGVTKWidgetPrivate(CGVTKWidget* q);
    ~CGVTKWidgetPrivate();

    void init();
    void configRenderer(vtkRenderer* renderer);
    void layoutRenderers();

    CGVTKWidget* q_ptr;
    QColor backgroundColor = Qt::black;
    bool multiViewports = false;
    vtkRenderer* defaultRenderer = nullptr;
    vtkOrientationMarkerWidget* orientationMarkerWidget = nullptr;

    QList<vtkRenderer*> renderers;
    QList<vtkProp*> actors;
    QList<vtkActor*> actors3d;
    QList<vtkProp*> props;

    double bounds[6];
};

CGVTKWidgetPrivate::CGVTKWidgetPrivate(CGVTKWidget *q) : q_ptr(q)
{
    init();
}

CGVTKWidgetPrivate::~CGVTKWidgetPrivate()
{

}

void CGVTKWidgetPrivate::init()
{
    layoutRenderers();
}

void CGVTKWidgetPrivate::configRenderer(vtkRenderer *renderer)
{
    if (!renderer)
        return;

    double bgclr[3];
    VTKUtils::vtkColor(backgroundColor, bgclr);

    renderer->SetBackground(bgclr);
}

void CGVTKWidgetPrivate::layoutRenderers()
{
    switch (renderers.size()) {
    case 1:
        CGVTKUtils::layoutRenderers<1>(renderers);
        break;

    case 2:
        CGVTKUtils::layoutRenderers<2>(renderers);
        break;

    case 3:
        CGVTKUtils::layoutRenderers<3>(renderers);
        break;

    case 4:
        CGVTKUtils::layoutRenderers<4>(renderers);
        break;

    case 5:
        CGVTKUtils::layoutRenderers<5>(renderers);
        break;

    case 6:
        CGVTKUtils::layoutRenderers<6>(renderers);
        break;

    case 7:
        CGVTKUtils::layoutRenderers<7>(renderers);
        break;

    case 8:
        CGVTKUtils::layoutRenderers<8>(renderers);
        break;

    case 9:
        CGVTKUtils::layoutRenderers<9>(renderers);
        break;

    case 10:
        CGVTKUtils::layoutRenderers<10>(renderers);
        break;

    default:
        CGVTKUtils::layoutRenderers<-1>(renderers);
    }
}


CGVTKWidget::CGVTKWidget(QWidget* parent) : QVTKWidget(parent)
{
    d_ptr = new CGVTKWidgetPrivate(this);
}

CGVTKWidget::~CGVTKWidget()
{
    delete d_ptr;
}

void CGVTKWidget::setMultiViewports(bool multi)
{
    if (d_ptr->multiViewports != multi)
    {
        d_ptr->multiViewports = multi;
    }
}

bool CGVTKWidget::multiViewports() const
{
    return d_ptr->multiViewports;
}

void CGVTKWidget::addActor(vtkProp *actor, const QColor &clr)
{
    if (!actor || d_ptr->actors.contains(actor))
        return;

    double vtkClr[3];
    VTKUtils::vtkColor(clr, vtkClr);

    d_ptr->actors.append(actor);

    if (!d_ptr->multiViewports) {
        if (d_ptr->renderers.isEmpty()) {
            vtkRenderer* renderer = vtkRenderer::New();
            renderer->SetBackground(vtkClr);
            d_ptr->configRenderer(renderer);
            renderer->AddActor(actor);
            GetRenderWindow()->AddRenderer(renderer);
            d_ptr->renderers.append(renderer);
            renderer->ResetCamera();
        } else {
            defaultRenderer()->SetBackground(vtkClr);
            defaultRenderer()->AddActor(actor);
        }
    } else {
        if (!defaultRendererTaken()) {
            defaultRenderer()->SetBackground(vtkClr);
            defaultRenderer()->AddActor(actor);
        } else {
            vtkRenderer* renderer = vtkRenderer::New();
            renderer->SetBackground(vtkClr);
            d_ptr->configRenderer(renderer);
            renderer->AddActor(actor);
            GetRenderWindow()->AddRenderer(renderer);
            d_ptr->renderers.append(renderer);
            d_ptr->layoutRenderers();
            renderer->ResetCamera();
        }
    }
}

void CGVTKWidget::addActor3D(vtkActor *actor3d, const QColor &clr)
{
    if (!actor3d || d_ptr->actors3d.contains(actor3d))
        return;

    double vtkClr[3];
    VTKUtils::vtkColor(clr, vtkClr);

    d_ptr->actors3d.append(actor3d);

    if (!d_ptr->multiViewports) {
        if (d_ptr->renderers.isEmpty()) {
            vtkRenderer* renderer = vtkRenderer::New();
            renderer->SetBackground(vtkClr);
            d_ptr->configRenderer(renderer);
            renderer->AddActor(actor3d);
            GetRenderWindow()->AddRenderer(renderer);
            d_ptr->renderers.append(renderer);
            renderer->ResetCamera();
        } else {
            defaultRenderer()->SetBackground(vtkClr);
            defaultRenderer()->AddActor(actor3d);
        }
    } else {
        if (!defaultRendererTaken()) {
            defaultRenderer()->SetBackground(vtkClr);
            defaultRenderer()->AddActor(actor3d);
        } else {
            vtkRenderer* renderer = vtkRenderer::New();
            renderer->SetBackground(vtkClr);
            d_ptr->configRenderer(renderer);
            renderer->AddActor(actor3d);
            GetRenderWindow()->AddRenderer(renderer);
            d_ptr->renderers.append(renderer);
            d_ptr->layoutRenderers();
            renderer->ResetCamera();
        }
    }
}

void CGVTKWidget::addViewProp(vtkProp *prop)
{
    if (!prop || d_ptr->props.contains(prop))
        return;

    d_ptr->props.append(prop);

    if (!d_ptr->multiViewports)
    {
        if (d_ptr->renderers.isEmpty()) {
            vtkRenderer* renderer = vtkRenderer::New();
            d_ptr->configRenderer(renderer);
            renderer->AddViewProp(prop);
            GetRenderWindow()->AddRenderer(renderer);
            d_ptr->renderers.append(renderer);
            renderer->ResetCamera();
        }
        else
        {
            defaultRenderer()->AddViewProp(prop);
        }
    }
    else
    {
        if (!defaultRendererTaken())
        {
            defaultRenderer()->AddViewProp(prop);
        }
        else
        {
            vtkRenderer* renderer = vtkRenderer::New();
            d_ptr->configRenderer(renderer);
            renderer->AddViewProp(prop);
            GetRenderWindow()->AddRenderer(renderer);
            d_ptr->renderers.append(renderer);
            d_ptr->layoutRenderers();
            renderer->ResetCamera();
        }
    }
}

QList<vtkProp *> CGVTKWidget::actors() const
{
    return d_ptr->actors;
}

QList<vtkActor *> CGVTKWidget::actors3d() const
{
    return d_ptr->actors3d;
}

void CGVTKWidget::setActorsVisible(bool visible)
{
    foreach (auto actor, d_ptr->actors)
        actor->SetVisibility(visible);
}

void CGVTKWidget::setActorVisible(vtkProp* actor, bool visible)
{
    actor->SetVisibility(visible);
}

bool CGVTKWidget::actorVisible(vtkProp* actor)
{
    return actor->GetVisibility();
}

void CGVTKWidget::setActors3DVisible(bool visible)
{
    foreach (auto actor, d_ptr->actors3d)
        actor->SetVisibility(visible);
}

void CGVTKWidget::setActor3DVisible(vtkActor *actor3d, bool visible)
{
    actor3d->SetVisibility(visible);
}

bool CGVTKWidget::actor3DVisible(vtkActor *actor3d)
{
    return actor3d->GetVisibility();
}

void CGVTKWidget::setBackgroundColor(const QColor& clr)
{
    if (d_ptr->backgroundColor != clr) {
        d_ptr->backgroundColor = clr;

        foreach (vtkRenderer* renderer, d_ptr->renderers)
            d_ptr->configRenderer(renderer);

#if 0
        vtkRendererCollection* renderers = GetRenderWindow()->GetRenderers();
        vtkRenderer* renderer = renderers->GetFirstRenderer();
        while (renderer) {
            renderer = renderers->GetNextItem();
        }
#endif
        update();
    }
}

QColor CGVTKWidget::backgroundColor() const
{
    return d_ptr->backgroundColor;
}

vtkRenderer* CGVTKWidget::defaultRenderer()
{
    CGVTKUtils::vtkInitOnce(&d_ptr->defaultRenderer);
    GetRenderWindow()->AddRenderer(d_ptr->defaultRenderer);
    if (!d_ptr->renderers.contains(d_ptr->defaultRenderer))
        d_ptr->renderers.append(d_ptr->defaultRenderer);
    return d_ptr->defaultRenderer;
}

bool CGVTKWidget::defaultRendererTaken() const
{
    if (!d_ptr->defaultRenderer)
        return false;
    return d_ptr->defaultRenderer->GetActors()->GetNumberOfItems() != 0;
}

void CGVTKWidget::showOrientationMarker(bool show)
{
    if (!d_ptr->orientationMarkerWidget)
    {
        VTK_CREATE(vtkAxesActor, axes);
        axes->SetShaftTypeToCylinder();

        d_ptr->orientationMarkerWidget = vtkOrientationMarkerWidget::New();
        d_ptr->orientationMarkerWidget->SetOutlineColor(0.9300, 0.5700, 0.1300);
        d_ptr->orientationMarkerWidget->SetOrientationMarker(axes);
        d_ptr->orientationMarkerWidget->SetInteractor(GetInteractor());
        d_ptr->orientationMarkerWidget->SetViewport(0.0, 0.0, 0.23, 0.23);
        d_ptr->orientationMarkerWidget->SetEnabled(1);
        d_ptr->orientationMarkerWidget->InteractiveOff();
    }
    d_ptr->orientationMarkerWidget->SetEnabled(show);

    update();
}

void CGVTKWidget::setBounds(double* bounds)
{
    VTKUtils::ArrayAssigner<double, 6> aa;
    aa(bounds, d_ptr->bounds);
}

double CGVTKWidget::xMin() const
{
    return d_ptr->bounds[0];
}

double CGVTKWidget::xMax() const
{
    return d_ptr->bounds[1];
}

double CGVTKWidget::yMin() const
{
    return d_ptr->bounds[2];
}

double CGVTKWidget::yMax() const
{
    return d_ptr->bounds[3];
}

double CGVTKWidget::zMin() const
{
    return d_ptr->bounds[4];
}

double CGVTKWidget::zMax() const
{
    return d_ptr->bounds[5];
}
