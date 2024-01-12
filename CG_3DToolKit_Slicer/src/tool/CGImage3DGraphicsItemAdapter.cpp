#include "CGImage3DGraphicsItemAdapter.h"
#include <vtkPlane.h>
#include <vtkPlanes.h>
#include <CGPropertiesRegulator.h>

CGImage3DGraphicsItemAdapter *CGImage3DGraphicsItemAdapter:: m_CGImage3DGraphicsItemAdapter = nullptr;

CGImage3DGraphicsItemAdapter::CGImage3DGraphicsItemAdapter(QObject *parent) : QObject(parent)
{

}

CGImage3DGraphicsItemAdapter *CGImage3DGraphicsItemAdapter::getInstance()
{
    if (!m_CGImage3DGraphicsItemAdapter)
    {
        m_CGImage3DGraphicsItemAdapter = new CGImage3DGraphicsItemAdapter();
    }
    return m_CGImage3DGraphicsItemAdapter;
}

void CGImage3DGraphicsItemAdapter::SendPlane(vtkPlane *plane)
{
    m_plane = plane;
    CGPropertiesRegulator::getInstance()->ShowPropertiesVTKPlane(plane);
}

void CGImage3DGraphicsItemAdapter::SendSphere(double* sphere)
{
    CGPropertiesRegulator::getInstance()->ShowPropertiesVTKSphere(sphere);
}

void CGImage3DGraphicsItemAdapter::SendPlanes(vtkPlanes *planes)
{
    m_planes = planes;
    CGPropertiesRegulator::getInstance()->ShowPropertiesVTKPlanes(planes);
}

vtkPlane *CGImage3DGraphicsItemAdapter::GetPlane() const
{
    return m_plane;
}

vtkPlanes* CGImage3DGraphicsItemAdapter::GetPlanes() const
{
    return m_planes;
}
