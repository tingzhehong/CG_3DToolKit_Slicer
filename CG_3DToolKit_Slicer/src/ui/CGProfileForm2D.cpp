#include "CGProfileForm2D.h"
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
#include <CGGraphicsLineItemHorizontal.h>
#include <CGGraphicsLineItemVertical.h>
#include <QDebug>

CGProfileForm2D::CGProfileForm2D(QWidget *parent) : QWidget(parent)
{
    m_pScene = new QGraphicsScene();
    m_pPixmap = new QPixmap();
    m_pItem = new QGraphicsPixmapItem();
    m_pGraphicsView = new CGGraphicsView(this);

    InitUi();
    InitConnections();
    InitTools();
}

void CGProfileForm2D::OnUseTool()
{
    if (!bGraphicsScene) return;

    RemoveTools();

    switch (m_CurrentToolType)
    {
    case TwoPointLineTool:
        InitTwoPointLineTool();
        m_pScene->addItem(m_pTwoPointLineTool);
        break;

    case RectTool:
        InitRectTool();
        m_pScene->addItem(m_pRectTool);
        break;

    case CircleTool:
        InitCircleTool();
        m_pScene->addItem(m_pCircleTool);
        break;

    case HorizontalLineTool:
        InitHorizontalLineTool();
        m_pScene->addItem(m_pHorizontalLineTool);
        break;

    case VerticalLineTool:
        InitVerticalLineTool();
        m_pScene->addItem(m_pVerticalLineTool);
        break;

    default:
        break;
    }
    m_LastToolType = m_CurrentToolType;
    m_pScene->update();
    m_pGraphicsView->RemoveFilter();

    IsTool = true;
}

void CGProfileForm2D::OnDelTool()
{
    RemoveTools();
}

void CGProfileForm2D::InitUi()
{
    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_pGraphicsView);

    setLayout(pMainLayout);
}

void CGProfileForm2D::InitConnections()
{

}

void CGProfileForm2D::InitTools()
{
    m_pTwoPointLineTool = new CGGraphicsLineItem();
    m_pRectTool = new CGGraphicsRectItem();
    m_pCircleTool = new CGGraphicsCircleItem();
    m_pHorizontalLineTool = new CGGraphicsLineItemHorizontal();
    m_pVerticalLineTool = new CGGraphicsLineItemVertical();
}

void CGProfileForm2D::RemoveTools()
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

        case HorizontalLineTool:
            m_pScene->removeItem(m_pHorizontalLineTool);
            break;

        case VerticalLineTool:
            m_pScene->removeItem(m_pVerticalLineTool);
            break;

        default:
            break;
        }
        m_pScene->update();
        m_pGraphicsView->InstallFilter();

        IsTool = false;
    }
}

void CGProfileForm2D::SetVerticalLine(double pos)
{
    int x = static_cast<int>(m_pPixmap->width() * pos);
    m_pVerticalLineTool->setLine(x, 0, x, m_pPixmap->height());
    m_Line.setLine(x, 0, x, m_pPixmap->height());
    CGImage2DGraphicsItemAdapter::getInstance()->SendLine(m_Line);
    CGImage2DGraphicsItemAdapter::getInstance()->m_Status = true;
    CGImage2DGraphicsItemAdapter::getInstance()->m_SendType = 0;
}

void CGProfileForm2D::SetHorizontalLine(double pos)
{
    int y = static_cast<int>(m_pPixmap->height() * pos);
    m_pHorizontalLineTool->setLine(0, y, m_pPixmap->width(), y);
    m_Line.setLine(0, y, m_pPixmap->width(), y);
    CGImage2DGraphicsItemAdapter::getInstance()->SendLine(m_Line);
    CGImage2DGraphicsItemAdapter::getInstance()->m_Status = true;
    CGImage2DGraphicsItemAdapter::getInstance()->m_SendType = 0;
}

void CGProfileForm2D::InitTwoPointLineTool()
{
    //!m_pTwoPointLineTool
    m_pTwoPointLineTool->setFlag(QGraphicsItem::ItemIsSelectable, true);
    m_pTwoPointLineTool->setFlag(QGraphicsItem::ItemIsMovable, true);
    m_pTwoPointLineTool->setFlag(QGraphicsItem::ItemIsFocusable, true);
    m_pTwoPointLineTool->setAcceptHoverEvents(true);
    QPen pen;
    pen.setCosmetic(true);
    pen.setColor(Qt::white);
    pen.setWidth(3);
    m_pTwoPointLineTool->setPen(pen);
    m_pTwoPointLineTool->setLine(m_pPixmap->width()/3*1, m_pPixmap->height()/2, m_pPixmap->width()/3*2, m_pPixmap->height()/2);
    m_Line.setLine(m_pPixmap->width()/3*1, m_pPixmap->height()/2, m_pPixmap->width()/3*2, m_pPixmap->height()/2);
}

void CGProfileForm2D::InitRectTool()
{
    //!m_pRectTool
    m_pRectTool->setFlag(QGraphicsItem::ItemIsSelectable, true);
    m_pRectTool->setFlag(QGraphicsItem::ItemIsMovable, true);
    m_pRectTool->setFlag(QGraphicsItem::ItemIsFocusable, true);
    m_pRectTool->setAcceptHoverEvents(true);
    QPen pen;
    pen.setCosmetic(true);
    pen.setColor(Qt::white);
    pen.setWidth(3);
    m_pRectTool->setPen(pen);
    m_pRectTool->setRect(m_pPixmap->width()/3, m_pPixmap->height()/3, m_pPixmap->width()/5, m_pPixmap->height()/5);
    m_Rect.setRect(m_pPixmap->width()/3, m_pPixmap->height()/3, m_pPixmap->width()/5, m_pPixmap->height()/5);
}

void CGProfileForm2D::InitCircleTool()
{
    //!m_pCircleTool
    m_pCircleTool->setFlag(QGraphicsItem::ItemIsSelectable, true);
    m_pCircleTool->setFlag(QGraphicsItem::ItemIsMovable, true);
    m_pCircleTool->setFlag(QGraphicsItem::ItemIsFocusable, true);
    m_pCircleTool->setAcceptHoverEvents(true);
    QPen pen;
    pen.setCosmetic(true);
    pen.setColor(Qt::white);
    pen.setWidth(3);
    m_pCircleTool->setPen(pen);
    m_pCircleTool->setRect(m_pPixmap->width()/3, m_pPixmap->width()/3, m_pPixmap->width()/5, m_pPixmap->width()/5);
    m_Rect.setRect(m_pPixmap->width()/3, m_pPixmap->width()/3, m_pPixmap->width()/5, m_pPixmap->width()/5);
}

void CGProfileForm2D::InitHorizontalLineTool()
{
    //!m_pHorizontalLineTool
    m_pHorizontalLineTool->setFlag(QGraphicsItem::ItemIsSelectable, true);
    m_pHorizontalLineTool->setFlag(QGraphicsItem::ItemIsSelectable, true);
    m_pHorizontalLineTool->setFlag(QGraphicsItem::ItemIsMovable, true);
    m_pHorizontalLineTool->setFlag(QGraphicsItem::ItemIsFocusable, true);
    m_pHorizontalLineTool->setAcceptHoverEvents(true);
    QPen pen;
    pen.setCosmetic(true);
    pen.setColor(Qt::white);
    pen.setWidth(3);
    m_pHorizontalLineTool->setPen(pen);
    m_pHorizontalLineTool->setLine(0, m_pPixmap->height()/2, m_pPixmap->width(), m_pPixmap->height()/2);
    m_Line.setLine(0, m_pPixmap->height()/2, m_pPixmap->width(), m_pPixmap->height()/2);
}

void CGProfileForm2D::InitVerticalLineTool()
{
    //!m_pVerticalLineTool
    m_pVerticalLineTool->setFlag(QGraphicsItem::ItemIsSelectable, true);
    m_pVerticalLineTool->setFlag(QGraphicsItem::ItemIsSelectable, true);
    m_pVerticalLineTool->setFlag(QGraphicsItem::ItemIsMovable, true);
    m_pVerticalLineTool->setFlag(QGraphicsItem::ItemIsFocusable, true);
    m_pVerticalLineTool->setAcceptHoverEvents(true);
    QPen pen;
    pen.setCosmetic(true);
    pen.setColor(Qt::white);
    pen.setWidth(3);
    m_pVerticalLineTool->setPen(pen);
    m_pVerticalLineTool->setLine(m_pPixmap->width()/2, 0, m_pPixmap->width()/2, m_pPixmap->height());
    m_Line.setLine(m_pPixmap->width()/2, 0, m_pPixmap->width()/2, m_pPixmap->height());
}
