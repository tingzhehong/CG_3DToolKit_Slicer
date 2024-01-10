#include "CGShapeLineItem.h"

CGShapeLineItem::CGShapeLineItem(qreal x1, qreal y1, qreal x2, qreal y2) : CGShapeBaseItem(QPointF((x1 + x2) / 2, (y1 + y2) / 2), Line)
{
    ControlList << new CGShapeControlItem(this, center, 0);
    ControlList << new CGShapeControlItem(this, QPointF(x1, y1), 1);
    ControlList << new CGShapeControlItem(this, QPointF(x2, y2), 2);
    P1 = QPointF(x1, y1);
    P2 = QPointF(x2, y2);
}

void CGShapeLineItem::SetLine(CGShapeLine line)
{
    ControlList[0]->SetPoint(QPointF((line.p1_x + line.p2_x) / 2, (line.p1_y + line.p2_y) / 2));
    ControlList[1]->SetPoint(QPointF(line.p1_x, line.p1_y));
    ControlList[2]->SetPoint(QPointF(line.p2_x, line.p2_y));
    this->setFocus();
}

void CGShapeLineItem::GetLine(CGShapeLine &line)
{
    QPointF P1 = this->mapToScene(ControlList[1]->x(), ControlList[1]->y());
    QPointF P2 = this->mapToScene(ControlList[2]->x(), ControlList[2]->y());

    line.p1_x = P1.x();
    line.p1_y = P1.y();
    line.p2_x = P2.x();
    line.p2_y = P2.y();
}

bool CGShapeLineItem::UpDate(int index)
{
    if (index == 1)
    {
        P1 = ControlList[1]->GetPoint();
    }
    else if (index == 2)
    {
        P2 = ControlList[2]->GetPoint();
    }
    ControlList[0]->SetPoint(QPointF((P1.x() + P2.x()) / 2, (P1.y() + P2.y()) / 2));
    ControlList[1]->SetPoint(QPointF(P1.x(), P1.y()));
    ControlList[2]->SetPoint(QPointF(P2.x(), P2.y()));
    return true;
}

void CGShapeLineItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    CGShapeBaseItem::paint(painter, option, widget);
    painter->drawLine(P1, P2);
}
