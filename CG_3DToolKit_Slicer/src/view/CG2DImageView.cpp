#include "CG2DImageView.h"
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsPixmapItem>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <CGGraphicsLineItem.h>
#include <CGGraphicsRectItem.h>
#include <CGGraphicsCircleItem.h>
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
    InitTools();
    setWindowTitle(tr(u8"2D  图像"));
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
}

void CG2DImageView::OnDelTool()
{
    RemoveTools();
}

void CG2DImageView::OnUseTool()
{
    if (!bGraphicsScene) return;

    RemoveTools();

    switch (m_CurrentToolType)
    {
    case TwoPointLineTool:
        InitTwoPointLineTool();
        TwoPointLineProfileHandle();
        m_pScene->addItem(m_pTwoPointLineTool);
        break;

    case RectTool:
        InitRectTool();
        RectProfileHandle();
        m_pScene->addItem(m_pRectTool);
        break;

    case CircleTool:
        InitCircleTool();
        CircleProfileHandle();
        m_pScene->addItem(m_pCircleTool);
        break;

    default:
        break;
    }
    m_LastToolType = m_CurrentToolType;
    m_pScene->update();
    m_pGraphicsView->RemoveFilter();

    IsTool = true;
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

void CG2DImageView::InitTools()
{
    m_pTwoPointLineTool = new CGGraphicsLineItem();
    m_pRectTool = new CGGraphicsRectItem();
    m_pCircleTool = new CGGraphicsCircleItem();
}

void CG2DImageView::RemoveTools()
{
    if (IsTool)
    {
        switch (m_LastToolType)
        {
        case TwoPointLineTool:
            m_pScene->removeItem(m_pTwoPointLineTool);
            break;

        case RectTool:
            m_pScene->removeItem(m_pRectTool);
            break;

        case CircleTool:
            m_pScene->removeItem(m_pCircleTool);
            break;

        default:
            break;
        }
        m_pScene->update();
        m_pGraphicsView->InstallFilter();

        IsTool = false;
    }
}

QPixmap* CG2DImageView::GetPixmap() const
{
    return m_pPixmap;
}

void CG2DImageView::InitTwoPointLineTool()
{
    //!m_pTwoPointLineTool
    m_pTwoPointLineTool->setFlag(QGraphicsItem::ItemIsSelectable, true);
    m_pTwoPointLineTool->setFlag(QGraphicsItem::ItemIsMovable, true);
    m_pTwoPointLineTool->setFlag(QGraphicsItem::ItemIsFocusable, true);
    m_pTwoPointLineTool->setAcceptHoverEvents(true);
    QPen pen;
    pen.setCosmetic(true);
    pen.setColor(Qt::magenta);
    pen.setWidth(3);
    m_pTwoPointLineTool->setPen(pen);
    m_pTwoPointLineTool->setLine(m_pPixmap->width()/3*1, m_pPixmap->height()/2, m_pPixmap->width()/3*2, m_pPixmap->height()/2);
    m_Line.setLine(m_pPixmap->width()/3*1, m_pPixmap->height()/2, m_pPixmap->width()/3*2, m_pPixmap->height()/2);
}

void CG2DImageView::InitRectTool()
{
    //!m_pRectTool
    m_pRectTool->setFlag(QGraphicsItem::ItemIsSelectable, true);
    m_pRectTool->setFlag(QGraphicsItem::ItemIsMovable, true);
    m_pRectTool->setFlag(QGraphicsItem::ItemIsFocusable, true);
    m_pRectTool->setAcceptHoverEvents(true);
    QPen pen;
    pen.setCosmetic(true);
    pen.setColor(Qt::magenta);
    pen.setWidth(3);
    m_pRectTool->setPen(pen);
    m_pRectTool->setRect(m_pPixmap->width()/3, m_pPixmap->height()/3, m_pPixmap->width()/5, m_pPixmap->height()/5);
    m_Rect.setRect(m_pPixmap->width()/3, m_pPixmap->height()/3, m_pPixmap->width()/5, m_pPixmap->height()/5);
}

void CG2DImageView::InitCircleTool()
{
    //!m_pCircleTool
    m_pCircleTool->setFlag(QGraphicsItem::ItemIsSelectable, true);
    m_pCircleTool->setFlag(QGraphicsItem::ItemIsMovable, true);
    m_pCircleTool->setFlag(QGraphicsItem::ItemIsFocusable, true);
    m_pCircleTool->setAcceptHoverEvents(true);
    QPen pen;
    pen.setCosmetic(true);
    pen.setColor(Qt::magenta);
    pen.setWidth(3);
    m_pCircleTool->setPen(pen);
    m_pCircleTool->setRect(m_pPixmap->width()/3, m_pPixmap->width()/3, m_pPixmap->width()/5, m_pPixmap->width()/5);
    m_Rect.setRect(m_pPixmap->width()/3, m_pPixmap->width()/3, m_pPixmap->width()/5, m_pPixmap->width()/5);
}

void CG2DImageView::TwoPointLineProfileHandle()
{

}

void CG2DImageView::RectProfileHandle()
{

}

void CG2DImageView::CircleProfileHandle()
{

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
