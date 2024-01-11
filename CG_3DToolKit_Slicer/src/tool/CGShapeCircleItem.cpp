#include "CGShapeCircleItem.h"

CGShapeCircleItem::CGShapeCircleItem(qreal x, qreal y, qreal R) : CGShapeBaseItem(QPointF(x, y), Circle)
{
    Radius = R;
    ControlList << new CGShapeControlItem(this, center, 0);
    ControlList << new CGShapeControlItem(this, center + QPointF(R, 0), 1);
}

void CGShapeCircleItem::GetCircle(CGShapeCircle &Cir)
{
    QPointF GetRec = this->mapToScene(ControlList[0]->x(), ControlList[0]->y());
    Cir.col = GetRec.x();
    Cir.row = GetRec.y();
    Cir.radius = Radius;
}

QRectF CGShapeCircleItem::boundingRect() const
{
    return QRectF(center.x() - Radius, center.y() - Radius, Radius * 2, Radius * 2);
}

bool CGShapeCircleItem::UpDate(int index)
{
    QPointF Pf = ControlList[index]->GetPoint();
    QPointF tmp = Pf - center;
    Radius = sqrt(tmp.x() * tmp.x() + tmp.y() * tmp.y());
    return true;
}

void CGShapeCircleItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    CGShapeBaseItem::paint(painter, option, widget);
    painter->drawEllipse(QRectF(center.x() - Radius, center.y() - Radius, Radius * 2, Radius * 2));
}
