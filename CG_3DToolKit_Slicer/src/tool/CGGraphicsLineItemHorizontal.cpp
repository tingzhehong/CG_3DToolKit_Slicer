#include "CGGraphicsLineItemHorizontal.h"
#include <QDebug>
#include <QCursor>
#include <QPainter>

CGGraphicsLineItemHorizontal::CGGraphicsLineItemHorizontal()
{

}

void CGGraphicsLineItemHorizontal::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
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

void CGGraphicsLineItemHorizontal::wheelEvent(QGraphicsSceneWheelEvent *event)
{
    Q_UNUSED(event);
}

void CGGraphicsLineItemHorizontal::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
		m_Line = line();
		m_PosPointF = event->pos();
		
		qreal x1 = m_Line.x1();
        qreal y1 = m_Line.y1();
        qreal x2 = m_Line.x2();
        qreal y2 = m_Line.y2();

        if (y1 - m_HitRange <=  m_PosPointF.y() && m_PosPointF.y() <= y2 + m_HitRange)
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

void CGGraphicsLineItemHorizontal::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    bPressHit = false;
    setCursor(QCursor(Qt::OpenHandCursor));
    Q_UNUSED(event);

    CGImage2DGraphicsItemAdapter::getInstance()->m_Status = false;
}

void CGGraphicsLineItemHorizontal::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    m_CurrentPointF = event->pos();

    if (bPressHit)
    {
        qreal x1 = (0);
        qreal y1 = (event->pos().y());
        qreal x2 = (m_LastPointF.x());
        qreal y2 = (event->pos().y());

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

void CGGraphicsLineItemHorizontal::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    setCursor(QCursor(Qt::OpenHandCursor));
    Q_UNUSED(event);
}

void CGGraphicsLineItemHorizontal::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
    setCursor(QCursor(Qt::OpenHandCursor));
    Q_UNUSED(event);
}
