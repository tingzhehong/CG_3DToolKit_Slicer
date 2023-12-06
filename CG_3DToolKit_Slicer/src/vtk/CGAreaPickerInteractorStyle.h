#ifndef CGAREAPICKERINTERACTORSTYLE_H
#define CGAREAPICKERINTERACTORSTYLE_H

#include <CGVTKHeader.h>
#include <vtkSmartPointer.h>
#include <vtkPointPicker.h>
#include <vtkAreaPicker.h>
#include <vtkExtractGeometry.h>
#include <vtkInteractorStyleRubberBandPick.h>

#define VTKISRBP_ORIENT 0
#define VTKISRBP_SELECT 1

class CGAreaPickerInteractorStyle : public vtkInteractorStyleRubberBandPick
{
public:
    static CGAreaPickerInteractorStyle* New();
    vtkTypeMacro(CGAreaPickerInteractorStyle, vtkInteractorStyleRubberBandPick)

    CGAreaPickerInteractorStyle()
    {
        this->SelectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
        this->SelectedActor = vtkSmartPointer<vtkActor>::New();
        this->SelectedActor->SetMapper(SelectedMapper);
    }

    virtual void OnLeftButtonUp()
    {
        // Forward events
        vtkInteractorStyleRubberBandPick::OnLeftButtonUp();

        vtkPlanes* frustum = static_cast<vtkAreaPicker*>(this->GetInteractor()->GetPicker())->GetFrustum();

        vtkSmartPointer<vtkExtractGeometry> extractGeometry = vtkSmartPointer<vtkExtractGeometry>::New();
        extractGeometry->SetImplicitFunction(frustum);
        extractGeometry->SetInputData(this->Points);
        extractGeometry->Update();

        vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        glyphFilter->SetInputConnection(extractGeometry->GetOutputPort());
        glyphFilter->Update();

        vtkPolyData* selected = glyphFilter->GetOutput();
        std::cout << "Selected " << selected->GetNumberOfPoints() << " points" << std::endl;
        std::cout << "Selected " << selected->GetNumberOfCells() << " cells" << std::endl;

        this->SelectedMapper->SetInputData(selected);
        this->SelectedMapper->ScalarVisibilityOff();

        this->SelectedActor->GetProperty()->SetColor(1.0, 0.0, 0.0);
        this->SelectedActor->GetProperty()->SetPointSize(3);

        this->CurrentRenderer->AddActor(SelectedActor);
        this->GetInteractor()->GetRenderWindow()->Render();
        this->HighlightProp(NULL);
    }

    void SetPoints(vtkSmartPointer<vtkPolyData> points) { this->Points = points; }
    void SetCurrentMode(int mode = 0) { this->CurrentMode = mode; }

private:
    vtkSmartPointer<vtkPolyData> Points;
    vtkSmartPointer<vtkActor> SelectedActor;
    vtkSmartPointer<vtkDataSetMapper> SelectedMapper;
};

#endif // CGAREAPICKERINTERACTORSTYLE_H
