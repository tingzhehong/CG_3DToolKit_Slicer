#ifndef CGPOINTPICKERINTERACTORSTYLE_H
#define CGPOINTPICKERINTERACTORSTYLE_H

#include <CGVTKHeader.h>
#include <vtkSmartPointer.h>
#include <vtkPointPicker.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

class PointPickerInteractorStyle : public vtkInteractorStyleTrackballCamera
{

public:
    static PointPickerInteractorStyle* New();
    vtkTypeMacro(PointPickerInteractorStyle, vtkInteractorStyleTrackballCamera)

    virtual void OnLeftButtonDown()
    {
        vtkPointPicker* point_picker = vtkPointPicker::SafeDownCast(iren->GetPicker());

        if (!point_picker)
        {
            std::cout<< "Point picker not available, not selecting any points!\n";
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

            std::cout << x << " " << y << " " << z << std::endl;
        }

        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

    void SetPointPickerInteractor(vtkRenderWindowInteractor* interactor)
    {
        iren = interactor;
    }

private:
    float x, y, z;
    vtkSmartPointer<vtkRenderWindowInteractor> iren;
};

#endif // CGPOINTPICKERINTERACTORSTYLE_H
