#include "CGGraphicsCircleItem.h"
#include <QDebug>
#include <QCursor>

CGGraphicsCircleItem::CGGraphicsCircleItem()
{

}

void CGGraphicsCircleItem::wheelEvent(QGraphicsSceneWheelEvent *event)
{
    m_Ellipse = rect();

    qreal x = m_Ellipse.x();
    qreal y = m_Ellipse.y();
    qreal w = m_Ellipse.width();
    qreal h = m_Ellipse.height();
    qreal a = x + w / 2;
    qreal b = y + h / 2;

    qreal ww = 0, hh = 0, xx = 0, yy = 0;

    if (event->delta() > 0)
    {
        ww = w + 10.1;
        hh = h + 10.1;
        xx = a - ww / 2;
        yy = b - hh / 2;
    }
    else
    {
        ww = w - 10.9;
        hh = h - 10.9;
        xx = a - ww / 2;
        yy = b - hh / 2;
    }

    setRect(xx, yy, ww, hh);
    CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(xx, yy, ww, hh));
    CGImage2DGraphicsItemAdapter::getInstance()->m_Status = true;
    CGImage2DGraphicsItemAdapter::getInstance()->m_SendType = 1;
}

void CGGraphicsCircleItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        m_Ellipse = rect();
        m_PosPointF = event->pos();

        qreal x = m_Ellipse.x();
        qreal y = m_Ellipse.y();
        qreal w = m_Ellipse.width();
        qreal h = m_Ellipse.height();

        if (x  + (w / 2) - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + (w / 2) + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + m_HitRange)
        {
            m_LastPointF = QPointF(x + w, y + h);
            bPressHit = true;
            iPointHit = 1;
        }
        else if (x + (w / 2) - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + (w / 2) + m_HitRange && y + h - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
        {
            m_LastPointF = QPointF(x, y);
            bPressHit = true;
            iPointHit = 2;
        }
        else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + m_HitRange && y + (h / 2) - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + (h / 2)  + m_HitRange)
        {
            m_LastPointF = QPointF(x + w, y);
            bPressHit = true;
            iPointHit = 3;
        }
        else if (x + w - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y + (h / 2) - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + (h / 2) + m_HitRange)
        {
            m_LastPointF = QPointF(x, y + h);
            bPressHit = true;
            iPointHit = 4;
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

void CGGraphicsCircleItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    bPressHit = false;
    setCursor(QCursor(Qt::OpenHandCursor));
    Q_UNUSED(event);

    CGImage2DGraphicsItemAdapter::getInstance()->m_Status = false;
}

void CGGraphicsCircleItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    m_CurrentPointF = event->pos();

    if (bPressHit)
    {
        qreal x1 = (m_LastPointF.x());
        qreal y1 = (m_LastPointF.y());
        qreal x2 = (event->pos().x());
        qreal y2 = (event->pos().y());

        float r1 = fabs(x1 - x2);
        float r2 = fabs(y1 - y2);

        switch (iPointHit)
        {
        case 1:
            setRect(QRectF(x2, y2, r2, r2));
            CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x2, y2, r2, r2));
            break;
        case 2:
            setRect(QRectF(x1, y1, r2, r2));
            CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x1, y1, r2, r2));
            break;
        case 3:
            setRect(QRectF(x2, y1, r1, r1));
            CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x2, y1, r1, r1));
            break;
        case 4:
            setRect(QRectF(x1, y2, r1, r1));
            CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x1, y2, r1, r1));
            break;
        default:
            break;
        }
        CGImage2DGraphicsItemAdapter::getInstance()->m_Status = true;
        CGImage2DGraphicsItemAdapter::getInstance()->m_SendType = 1;
    }
    else
    {
        qreal x = m_Ellipse.x();
        qreal y = m_Ellipse.y();
        qreal w = m_Ellipse.width();
        qreal h = m_Ellipse.height();

        qreal dx = m_CurrentPointF.x() - m_LastPointF.x();
        qreal dy = m_CurrentPointF.y() - m_LastPointF.y();
        setRect(QRectF(x + dx, y + dy, w, h));
        CGImage2DGraphicsItemAdapter::getInstance()->SendRect(QRectF(x + dx, y + dy, w, h));
        CGImage2DGraphicsItemAdapter::getInstance()->m_Status = true;
        CGImage2DGraphicsItemAdapter::getInstance()->m_SendType = 1;

//      moveBy(m_CurrentPointF.x() - m_LastPointF.x(), m_CurrentPointF.y() - m_LastPointF.y());
    }
}

void CGGraphicsCircleItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    m_Ellipse = rect();
    m_PosPointF = event->pos();

    qreal x = m_Ellipse.x();
    qreal y = m_Ellipse.y();
    qreal w = m_Ellipse.width();
    qreal h = m_Ellipse.height();

    if (x  + (w / 2) - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + (w / 2) + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeAllCursor));
    }
    else if (x + (w / 2) - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + (w / 2) + m_HitRange && y + h - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeAllCursor));
    }
    else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + m_HitRange && y + (h / 2) - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + (h / 2)  + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeAllCursor));
    }
    else if (x + w - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y + (h / 2) - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + (h / 2) + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeAllCursor));
    }
    else
    {
       setCursor(QCursor(Qt::OpenHandCursor));
    }
}

void CGGraphicsCircleItem::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
    m_Ellipse = rect();
    m_PosPointF = event->pos();

    qreal x = m_Ellipse.x();
    qreal y = m_Ellipse.y();
    qreal w = m_Ellipse.width();
    qreal h = m_Ellipse.height();

    if (x  + (w / 2) - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + (w / 2) + m_HitRange && y - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeAllCursor));
    }
    else if (x + (w / 2) - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + (w / 2) + m_HitRange && y + h - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + h + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeAllCursor));
    }
    else if (x - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + m_HitRange && y + (h / 2) - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + (h / 2)  + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeAllCursor));
    }
    else if (x + w - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x + w + m_HitRange && y + (h / 2) - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y + (h / 2) + m_HitRange)
    {
        setCursor(QCursor(Qt::SizeAllCursor));
    }
    else
    {
       setCursor(QCursor(Qt::OpenHandCursor));
    }
}
