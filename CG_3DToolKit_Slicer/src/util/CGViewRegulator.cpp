#include "CGViewRegulator.h"
#include <CG2DImageView.h>
#include <CG3DImageView.h>
#include <CGNodeView.h>
#include <CGProfileView.h>
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

void CGViewRegulator::OnNodeViewRequest()
{

}

void CGViewRegulator::OnProfileViewRequest()
{
    auto pPixmap = m_CG2DImageView->GetPixmap();
    auto pActor = m_CG3DImageView->GetActor();
    m_CGProfileView->pPixmap = pPixmap;
    m_CGProfileView->pActor = pActor;
    m_CGProfileView->Apply();
}
