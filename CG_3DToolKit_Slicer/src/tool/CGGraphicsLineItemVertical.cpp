#include "CGGraphicsLineItemVertical.h"
#include <QDebug>
#include <QCursor>

CGGraphicsLineItemVertical::CGGraphicsLineItemVertical()
{

}

void CGGraphicsLineItemVertical::wheelEvent(QGraphicsSceneWheelEvent *event)
{
    Q_UNUSED(event);
}

void CGGraphicsLineItemVertical::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
		m_Line = line();
		m_PosPointF = event->pos();
		
		qreal x1 = m_Line.x1();
        qreal y1 = m_Line.y1();
        qreal x2 = m_Line.x2();
        qreal y2 = m_Line.y2();

        if (x1 - m_HitRange <=  m_PosPointF.x() && m_PosPointF.x() <= x2 + m_HitRange)
		{
            m_LastPointF = QPointF(x2, y2);
			bPressHit = true;
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

void CGGraphicsLineItemVertical::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    bPressHit = false;
    setCursor(QCursor(Qt::OpenHandCursor));
    Q_UNUSED(event);

    CGImage2DGraphicsItemAdapter::getInstance()->m_Status = false;
}

void CGGraphicsLineItemVertical::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    m_CurrentPointF = event->pos();

    if (bPressHit)
    {
        qreal x1 = (event->pos().x());
        qreal y1 = (0);
        qreal x2 = (event->pos().x());
        qreal y2 = (m_LastPointF.y());

        setLine(x1, y1, x2, y2);
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

void CGGraphicsLineItemVertical::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    setCursor(QCursor(Qt::OpenHandCursor));
    Q_UNUSED(event);
}

void CGGraphicsLineItemVertical::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
    setCursor(QCursor(Qt::OpenHandCursor));	
    Q_UNUSED(event);
}
