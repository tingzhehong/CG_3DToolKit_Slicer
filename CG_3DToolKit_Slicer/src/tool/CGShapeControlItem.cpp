#include "CGShapeControlItem.h"
#include "CGShapeBaseItem.h"
#include <QCursor>
#include <QDebug>

CGShapeControlItem::CGShapeControlItem(QGraphicsItemGroup* parent, QPointF p, int type) : QAbstractGraphicsShapeItem(parent),
    m_Point(p),
    m_PointType(type)
{
    setPos(m_Point);

    if (m_PointType == 0)
    {
        setCursor(QCursor(Qt::OpenHandCursor));
    }
    else
    {
        setCursor(QCursor(Qt::PointingHandCursor));
        setVisible(false);
    }
    setBrush(QBrush(QColor(255, 0, 255)));
    setCacheMode(DeviceCoordinateCache);
    setFlags(QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsMovable | ItemStopsFocusHandling);

    m_Rect = QRectF(-CGShapeBaseItem::ControlSize, -CGShapeBaseItem::ControlSize, CGShapeBaseItem::ControlSize * 2, CGShapeBaseItem::ControlSize * 2);
    m_Pen.setWidthF(0);
    m_Pen.setCosmetic(true);
}

QRectF CGShapeControlItem::boundingRect() const
{
    return m_Rect;
}

void CGShapeControlItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setPen(m_Pen);
    painter->setBrush(brush());
    setPos(m_Point);

    if (m_PointType == 0) //0号点默认为中心点
        painter->drawRect(m_Rect);
    else
        painter->drawEllipse(m_Rect);
}

void CGShapeControlItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->buttons() == Qt::LeftButton)
    {
        dx = event->scenePos().x() - event->lastScenePos().x();
        dy = event->scenePos().y() - event->lastScenePos().y();

        CGShapeBaseItem* item = static_cast<CGShapeBaseItem*>(this->parentItem());

        if (m_PointType == 0)
        {
            item->moveBy(dx, dy);
        }
        else
        {
            //记录上一次坐标结果
            m_LastPoint = m_Point;
            m_Point = this->mapToParent(event->pos());
            //更新结果
            bool flg = item->UpDate(m_PointType);
            if (flg)
            {
                //结果正常、更新位置
                setPos(m_Point);
            }
            else
            {
                //结果异常、退回上一次的位置
                m_Point = m_LastPoint;
                setPos(m_Point);
            }
        }
    }
}

void CGShapeControlItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
    if (m_PointType == 0)
    {
        //点击中心点时、激活编辑
        CGShapeBaseItem* item = static_cast<CGShapeBaseItem*>(this->parentItem());
        item->setFocus();
    }
    QGraphicsItem::mousePressEvent(event);
}
