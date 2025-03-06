#include "CGPointCoordinatesObserver.h"
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPointPicker.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>


namespace CGVTKUtils
{

CGPointCoordinatesObserver::CGPointCoordinatesObserver(QObject *parent) : QObject(parent), _X(0), _Y(0), _Z(0)
{

}

void CGPointCoordinatesObserver::Execute(vtkObject *caller, unsigned long eventid, void *)
{
    vtkRenderWindowInteractor *iren = reinterpret_cast<vtkRenderWindowInteractor*>(caller);

    if (eventid == vtkCommand::MouseMoveEvent)
    {
        SinglePick(iren, _X, _Y, _Z);
    }
}

void CGPointCoordinatesObserver::SinglePick(vtkRenderWindowInteractor *iren, float &x, float &y, float &z)
{
    vtkPointPicker* point_picker = vtkPointPicker::SafeDownCast(iren->GetPicker());

    if (!point_picker)
    {
        //qDebug() << "Point picker not available, not selecting any points!\n";
        return;
    }

    int mouse_x = iren->GetEventPosition()[0];
    int mouse_y = iren->GetEventPosition()[1];

    iren->StartPickCallback();
    vtkRenderer *ren = iren->FindPokedRenderer(mouse_x, mouse_y);
    point_picker->Pick(mouse_x, mouse_y, 0.0, ren);

    int idx = static_cast<int> (point_picker->GetPointId());
    if (point_picker->GetDataSet())
    {

        double p[3];
        point_picker->GetDataSet()->GetPoint(idx, p);
        x = float(p[0]); y = float(p[1]); z = float(p[2]);

        emit SignalPoint(x, y, z);
        //qDebug() << x << " " << y << " " << z;
    }
}

}
