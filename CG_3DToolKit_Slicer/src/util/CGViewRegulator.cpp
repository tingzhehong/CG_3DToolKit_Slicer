#include "CGViewRegulator.h"
#include <CG2DImageView.h>
#include <CG3DImageView.h>
#include <CGNodeView.h>
#include <CGProfileView.h>
#include <CGPointCloud.h>
#include <CGImageFormatConvert.h>
#include <QDebug>


CGViewRegulator::CGViewRegulator(QObject *parent)
    : QObject(parent)
    , m_CG2DImageView(nullptr)
    , m_CG3DImageView(nullptr)
    , m_CGNodeView(nullptr)
    , m_CGProfileView(nullptr)
{

}

CGViewRegulator::~CGViewRegulator()
{
    delete m_CG2DImageView; m_CG2DImageView = nullptr;
    delete m_CG3DImageView; m_CG3DImageView = nullptr;
    delete m_CGNodeView;    m_CGNodeView = nullptr;
    delete m_CGProfileView; m_CGProfileView = nullptr;
}

void CGViewRegulator::On2DImageViewRequest()
{

}

void CGViewRegulator::On3DImageViewRequest()
{

}

void CGViewRegulator::OnNodeViewRequest(const int type)
{
    if (type == 2) {
        if (g_Image.ColorImage.empty()) return;
        QImage qColorImage = CG::CVMat2QImage(g_Image.ColorImage);
        QPixmap qPixmap = QPixmap::fromImage(qColorImage, Qt::AutoColor);
        m_CG2DImageView->LoadImages(qPixmap);
    }
    if (type == 3) {
        if (g_PointCloud->empty()) return;
        auto actor = m_CG3DImageView->m_Actor;
        CG::VTKPointCloudIntensity(g_PointCloud, actor);
        m_CG3DImageView->CreatCubeAxes();
        m_CG3DImageView->CreatXYGrids(actor->GetBounds());
        m_CG3DImageView->m_CGVTKWidget->addActor3D(actor, QColor(25, 50, 75));
        m_CG3DImageView->m_CGVTKWidget->defaultRenderer()->ResetCamera();
        m_CG3DImageView->m_CGVTKWidget->update();
    }
}

void CGViewRegulator::OnProfileViewRequest()
{
    auto pPixmap = m_CG2DImageView->GetPixmap();
    auto pActor = m_CG3DImageView->GetActor();
    m_CGProfileView->pPixmap = pPixmap;
    m_CGProfileView->pActor = pActor;
    m_CGProfileView->Apply();
}

void CGViewRegulator::OnProfileViewSectionToolRemoved()
{
    m_CG3DImageView->InitPointPick();
}
