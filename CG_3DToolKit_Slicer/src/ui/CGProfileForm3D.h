#ifndef CGPROFILEFORM3D_H
#define CGPROFILEFORM3D_H

#include <QWidget>
#include <CGVTKWidget.h>
#include <vtkSmartPointer.h>

class vtkActor;
class CGProfileForm3D : public QWidget
{
    Q_OBJECT

public:
    explicit CGProfileForm3D(QWidget *parent = nullptr);
    ~CGProfileForm3D() = default;

    void InitUi();
    void InitConnections();

public:
    CGVTKWidget *m_CGVTKWidget = nullptr;
    vtkSmartPointer<vtkActor> m_Actor = nullptr;

public:
    enum ToolType
    {
        SectionVerticalTool = 6,
        SectionHorizontalTool = 7
    };
};

#endif // CGPROFILEFORM3D_H
