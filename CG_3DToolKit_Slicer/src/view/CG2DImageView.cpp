#include "CG2DImageView.h"
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsPixmapItem>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>
#include <QDebug>

CG2DImageView::CG2DImageView(QWidget *parent): CGBaseWidget(parent)
{
    m_pScene = new QGraphicsScene();
    m_pPixmap = new QPixmap();
    m_pItem = new QGraphicsPixmapItem();
    m_pGraphicsView = new CGGraphicsView(this);

    InitUi();
    InitConnections();
    setWindowTitle(tr(u8"2D  图像"));
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
}

void CG2DImageView::InitUi()
{
    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_pGraphicsView);

    setLayout(pMainLayout);
}

void CG2DImageView::InitConnections()
{

}

void CG2DImageView::ClearImages()
{
    if (bGraphicsScene)
        m_pGraphicsView->RemoveALLItems();
}

QPixmap* CG2DImageView::GetPixmap() const
{
    return m_pPixmap;
}

void CG2DImageView::LoadImages(const QString FileName)
{
    if (bGraphicsScene)
    {
        m_pGraphicsView->RemoveALLItems();
    }

    m_pPixmap->load(FileName);
    m_pItem->setPixmap(*m_pPixmap);
    m_pScene->addItem(m_pItem);
    m_pGraphicsView->setScene(m_pScene);
    m_pGraphicsView->ImageWidth = m_pPixmap->width();
    m_pGraphicsView->ImageHeight = m_pPixmap->height();

    if (!bGraphicsScene)
    {
        m_pGraphicsView->ResetGraphicsView();
        bGraphicsScene = true;
    }
}

void CG2DImageView::LoadImages(const QPixmap Pixmap)
{
    if (bGraphicsScene)
    {
        m_pGraphicsView->RemoveALLItems();
    }

    m_pPixmap = new QPixmap(Pixmap);
    m_pItem->setPixmap(*m_pPixmap);
    m_pScene->addItem(m_pItem);
    m_pGraphicsView->setScene(m_pScene);
    m_pGraphicsView->ImageWidth = m_pPixmap->width();
    m_pGraphicsView->ImageHeight = m_pPixmap->height();

    if (!bGraphicsScene)
    {
        m_pGraphicsView->ResetGraphicsView();
        bGraphicsScene = true;
    }
}
