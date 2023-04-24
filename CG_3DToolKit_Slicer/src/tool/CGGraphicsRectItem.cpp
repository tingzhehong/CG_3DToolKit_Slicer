#include "CGGraphicsRectItem.h"
#include <QDebug>
#include <QCursor>

CGGraphicsRectItem::CGGraphicsRectItem()
{

}

void CGGraphicsRectItem::wheelEvent(QGraphicsSceneWheelEvent *event)
{
     Q_UNUSED(event);
}

void CGGraphicsRectItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        m_Rect = rect();
        m_PosPointF = event->pos();

        qreal x = m_Rect.x();
        qreal y = m_Rect.y();
        qreal w = m_Rect.width();
        qreal h = m_Rect.height();

        if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + m_HitRange)
        {
            m_LastPointF = QPointF(x + w, y + h);
            bPressHit = true;
            iPointHit = 1;
        }
        else if (x + w - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y + h - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
        {
            m_LastPointF = QPointF(x, y);
            bPressHit = true;
            iPointHit = 2;
        }
        else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + m_HitRange && y + h - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
        {
            m_LastPointF = QPointF(x + w, y);
            bPressHit = true;
            iPointHit = 3;
        }
        else if (x + w  - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + m_HitRange)
        {
            m_LastPointF = QPointF(x, y + h);
            bPressHit = true;
            iPointHit = 4;
        }
        else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
        {
            m_LastPointF = QPointF(x + w, y);
            bPressHit = true;
            iPointHit = 5;
        }
        else if (x + w - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
        {
            m_LastPointF = QPointF(x, y);
            bPressHit = true;
            iPointHit = 6;
        }
        else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + m_HitRange)
        {
            m_LastPointF = QPointF(x, y + h);
            bPressHit = true;
            iPointHit = 7;
        }
        else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y + h - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
        {
            m_LastPointF = QPointF(x, y);
            bPressHit = true;
            iPointHit = 8;
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

void CGGraphicsRectItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    bPressHit = false;
    setCursor(QCursor(Qt::OpenHandCursor));
    Q_UNUSED(event);

    CGImage2DGraphicsItemAdapter::getInstance()->m_Status = false;
}

void CGGraphicsRectItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
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
            setRect(QRectF(x2, y2, x1 - x2, y1 - y2));
            CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x2, y2, x1 - x2, y1 - y2));
            break;
        case 2:
            setRect(QRectF(x1, y1, x2 - x1, y2 - y1));
            CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x1, y1, x2 - x1, y2 - y1));
            break;
        case 3:
            setRect(QRectF(x2, y1, x1 - x2, y2 - y1));
            CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x2, y1, x1 - x2, y2 - y1));
            break;
        case 4:
            setRect(QRectF(x1, y2, x2 - x1, y1 - y2));
            CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x1, y2, x2 - x1, y1 - y2));
            break;
        case 5:
            setRect(QRectF(x2, y1, x1 - x2, m_Rect.height()));
            CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x2, y1, x1 - x2, m_Rect.height()));
            break;
        case 6:
            setRect(QRectF(x1, y1, x2 - x1, m_Rect.height()));
            CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x1, y1, x2 - x1, m_Rect.height()));
            break;
        case 7:
            setRect(QRectF(x1, y2, m_Rect.width(), y1 - y2));
            CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x1, y2, m_Rect.width(), y1 - y2));
            break;
        case 8:
            setRect(QRectF(x1, y1, m_Rect.width(), y2 - y1));
            CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x1, y1, m_Rect.width(), y2 - y1));
            break;
        default:
            break;
        }
        CGImage2DGraphicsItemAdapter::getInstance()->m_Status = true;
        CGImage2DGraphicsItemAdapter::getInstance()->m_SendType = 1;
    }
    else
    {
        qreal x = m_Rect.x();
        qreal y = m_Rect.y();
        qreal w = m_Rect.width();
        qreal h = m_Rect.height();

        qreal dx = m_CurrentPointF.x() - m_LastPointF.x();
        qreal dy = m_CurrentPointF.y() - m_LastPointF.y();
        setRect(QRectF(x + dx, y + dy, w, h));
        CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x + dx, y + dy, w, h));
        CGImage2DGraphicsItemAdapter::getInstance()->m_Status = true;
        CGImage2DGraphicsItemAdapter::getInstance()->m_SendType = 1;

//      moveBy(m_CurrentPointF.x() - m_LastPointF.x(), m_CurrentPointF.y() - m_LastPointF.y());
    }
}

void CGGraphicsRectItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    m_Rect = rect();
    m_PosPointF = event->pos();

    qreal x = m_Rect.x();
    qreal y = m_Rect.y();
    qreal w = m_Rect.width();
    qreal h = m_Rect.height();

    if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeFDiagCursor));
    }
    else if (x + w - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y + h - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeFDiagCursor));
    }
    else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + m_HitRange && y + h - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeBDiagCursor));
    }
    else if (x + w  - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeBDiagCursor));
    }
    else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeHorCursor));
    }
    else if (x + w - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeHorCursor));
    }
    else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeVerCursor));
    }
    else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y + h - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeVerCursor));
    }
    else
    {
       setCursor(QCursor(Qt::OpenHandCursor));
    }
}

void CGGraphicsRectItem::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
    m_Rect = rect();
    m_PosPointF = event->pos();

    qreal x = m_Rect.x();
    qreal y = m_Rect.y();
    qreal w = m_Rect.width();
    qreal h = m_Rect.height();

    if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeFDiagCursor));
    }
    else if (x + w - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y + h - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeFDiagCursor));
    }
    else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + m_HitRange && y + h - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeBDiagCursor));
    }
    else if (x + w  - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeBDiagCursor));
    }
    else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeHorCursor));
    }
    else if (x + w - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeHorCursor));
    }
    else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeVerCursor));
    }
    else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y + h - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeVerCursor));
    }
    else
    {
       setCursor(QCursor(Qt::OpenHandCursor));
    }
}
