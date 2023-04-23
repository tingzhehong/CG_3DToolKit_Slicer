#include "CGProfileForm3D.h"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QDebug>
#include <Utils.h>
#include <CGVTKUtils.h>
#include <CGVTKHeader.h>


CGProfileForm3D::CGProfileForm3D(QWidget *parent) : QWidget(parent)
{
    InitUi();
    InitConnections();
}

void CGProfileForm3D::InitUi()
{
    m_CGVTKWidget = new CGVTKWidget(this);
    m_Actor = vtkSmartPointer<vtkActor>::New();

    double clr[3];
    QColor defaultColor(25, 50, 75);
    VTKUtils::vtkColor(defaultColor, clr);
    m_CGVTKWidget->defaultRenderer()->SetBackground(clr);
    m_CGVTKWidget->showOrientationMarker();
    m_CGVTKWidget->defaultRenderer()->AddActor(m_Actor);
    m_CGVTKWidget->defaultRenderer()->ResetCamera();
    m_CGVTKWidget->update();

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_CGVTKWidget);

    setLayout(pMainLayout);
}

void CGProfileForm3D::InitConnections()
{

}
