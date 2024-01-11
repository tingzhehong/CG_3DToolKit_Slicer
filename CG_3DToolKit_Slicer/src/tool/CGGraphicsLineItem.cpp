#include "CGGraphicsLineItem.h"
#include <QDebug>
#include <QCursor>
#include <QPainter>

CGGraphicsLineItem::CGGraphicsLineItem()
{

}

void CGGraphicsLineItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QGraphicsLineItem::paint(painter, option, widget);

    painter->setBrush(QBrush(QColor(0, 255, 127)));
    painter->setPen(Qt::NoPen);

    QLineF line = this->line();
    QPointF center1(line.x1(), line.y1());
    QPointF center2(line.x2(), line.y2());
    painter->drawEllipse(center1, m_HitRange / 3, m_HitRange / 3);
    painter->drawEllipse(center2, m_HitRange / 3, m_HitRange / 3);
}

void CGGraphicsLineItem::wheelEvent(QGraphicsSceneWheelEvent *event)
{
    Q_UNUSED(event);
}

void CGGraphicsLineItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        m_Line = line();
        m_PosPointF = event->pos();

        qreal x1 = m_Line.x1();
        qreal y1 = m_Line.y1();
        qreal x2 = m_Line.x2();
        qreal y2 = m_Line.y2();

        if (x1 - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x1 + m_HitRange && y1 - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y1 + m_HitRange)
        {
            m_LastPointF = QPointF(x2, y2);
            bPressHit = true;
            iPointHit = 1;
        }
        else if (x2 - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x2 + m_HitRange && y2 - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y2 + m_HitRange)
        {
            m_LastPointF = QPointF(x1, y1);
            bPressHit = true;
            iPointHit = 2;
        }
        else
        {
            setCursor(QCursor(Qt::ClosedHandCursor));
        }

        if (!bPressHit)
        {
            m_LastPointF = m_PosPointF;
        }
    }
}

void CGGraphicsLineItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    bPressHit = false;
    setCursor(QCursor(Qt::OpenHandCursor));
    Q_UNUSED(event);

    CGImage2DGraphicsItemAdapter::getInstance()->m_Status = false;
}

void CGGraphicsLineItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    m_CurrentPointF = event->pos();

    if (bPressHit)
    {
        qreal x1 = (m_LastPointF.x());
        qreal y1 = (m_LastPointF.y());
        qreal x2 = (event->pos().x());
        qreal y2 = (event->pos().y());

        switch (iPointHit)
        {
        case 1:
            setLine(x1, y1, x2, y2);
            break;
        case 2:
            setLine(x1, y1, x2, y2);
            break;
        default:
            break;
        }
        CGImage2DGraphicsItemAdapter::getInstance()->SendLine(QLine(x1, y1, x2, y2));
        CGImage2DGraphicsItemAdapter::getInstance()->m_Status = true;
        CGImage2DGraphicsItemAdapter::getInstance()->m_SendType = 0;
    }
    else
    {       
        qreal x1 = m_Line.x1();
        qreal y1 = m_Line.y1();
        qreal x2 = m_Line.x2();
        qreal y2 = m_Line.y2();

        qreal dx = m_CurrentPointF.x() - m_LastPointF.x();
        qreal dy = m_CurrentPointF.y() - m_LastPointF.y();
        setLine(x1 + dx, y1 + dy, x2 + dx, y2 + dy);
        CGImage2DGraphicsItemAdapter::getInstance()->SendLine(QLine(x1 + dx, y1 + dy, x2 + dx, y2 + dy));
        CGImage2DGraphicsItemAdapter::getInstance()->m_Status = true;
        CGImage2DGraphicsItemAdapter::getInstance()->m_SendType = 0;

//      moveBy(m_CurrentPointF.x() - m_LastPointF.x(), m_CurrentPointF.y() - m_LastPointF.y());
    }
}

void CGGraphicsLineItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    m_Line = line();
    m_PosPointF = event->pos();

    qreal x1 = m_Line.x1();
    qreal y1 = m_Line.y1();
    qreal x2 = m_Line.x2();
    qreal y2 = m_Line.y2();

    if (x1 - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x1 + m_HitRange && y1 - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y1 + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeAllCursor));
    }
    else if (x2 - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x2 + m_HitRange && y2 - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y2 + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeAllCursor));
    }
    else
    {
        setCursor(QCursor(Qt::OpenHandCursor));
    }
}

void CGGraphicsLineItem::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
    m_Line = line();
    m_PosPointF = event->pos();

    qreal x1 = m_Line.x1();
    qreal y1 = m_Line.y1();
    qreal x2 = m_Line.x2();
    qreal y2 = m_Line.y2();

    if (x1 - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x1 + m_HitRange && y1 - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y1 + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeAllCursor));
    }
    else if (x2 - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x2 + m_HitRange && y2 - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y2 + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeAllCursor));
    }
    else
    {
        setCursor(QCursor(Qt::OpenHandCursor));
    }
}
