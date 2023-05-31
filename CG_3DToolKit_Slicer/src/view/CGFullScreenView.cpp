#include "CGFullScreenView.h"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>
#include <QDebug>
#include <QKeyEvent>
#include <CGVTKHeader.h>

CGFullScreenView::CGFullScreenView(QWidget *parent) : QWidget(parent), m_CG3DImageView(nullptr)
{
    InitUi();
    InitConnections();
    resize(QSize(400, 300));
    setWindowFlags(Qt::WindowCloseButtonHint);
    setWindowTitle(tr(u8"3D  图像"));
    setWindowIcon(QIcon(":/res/icon/varo.png"));
}

CGFullScreenView::~CGFullScreenView()
{
    delete m_CG3DImageView;
    m_CG3DImageView = nullptr;
}

void CGFullScreenView::InitUi()
{
    m_CG3DImageView = new CG3DImageView();

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_CG3DImageView);

    setLayout(pMainLayout);
}

void CGFullScreenView::InitConnections()
{

}

void CGFullScreenView::SetSceneView(CG3DImageView *scene)
{
    m_CG3DImageView->m_CGVTKWidget->addActor3D(scene->m_Actor, QColor(25, 50, 75));
    m_CG3DImageView->m_CGVTKWidget->defaultRenderer()->ResetCamera();
    m_CG3DImageView->m_CGVTKWidget->update();
}

CG3DImageView *CGFullScreenView::GetSceneView() const
{
    return m_CG3DImageView;
}

void CGFullScreenView::ClearSceneView()
{
    int num = m_CG3DImageView->m_CGVTKWidget->actors3d().count();
    for (int i = 0; i < num; ++i)
    {
        m_CG3DImageView->m_CGVTKWidget->defaultRenderer()->RemoveActor(m_CG3DImageView->m_CGVTKWidget->actors3d()[i]);
    }
    m_CG3DImageView->m_CGVTKWidget->update();
}

void CGFullScreenView::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_F11)
        this->close();
}

void CGFullScreenView::closeEvent(QCloseEvent *event)
{
    ClearSceneView();

    Q_UNUSED(event);
}
