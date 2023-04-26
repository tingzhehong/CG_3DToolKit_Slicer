#include "CGPointPickObserver.h"

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

CGPointPickObserver::CGPointPickObserver(QObject *parent) : QObject(parent), X(0), Y(0), Z(0)
{

}

CGPointPickObserver::~CGPointPickObserver()
{

}

void CGPointPickObserver::Execute(vtkObject *caller, unsigned long eventid, void *)
{
    vtkRenderWindowInteractor *iren = reinterpret_cast<vtkRenderWindowInteractor*>(caller);

    if (eventid == vtkCommand::LeftButtonPressEvent)
    {
        SinglePick(iren, X, Y, Z);
    }
}

void CGPointPickObserver::SinglePick(vtkRenderWindowInteractor *iren, float &x, float &y, float &z)
{
    vtkPointPicker* point_picker = vtkPointPicker::SafeDownCast(iren->GetPicker());

    if (!point_picker)
    {
        printf("Point picker not available, not selecting any points!\n");
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
    }
}

}
