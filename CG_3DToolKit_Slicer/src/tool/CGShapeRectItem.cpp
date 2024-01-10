#include "CGShapeRectItem.h"

CGShapeRectItem::CGShapeRectItem(qreal x, qreal y, qreal width, qreal height) : CGShapeBaseItem(QPointF(x, y), Rectangle)
{
    ControlList << new CGShapeControlItem(this, QPointF(x + width / 2, y + height / 2), 0);
    ControlList << new CGShapeControlItem(this, QPointF(x, y), 1);
    ControlList << new CGShapeControlItem(this, QPointF(x + width, y), 2);
    ControlList << new CGShapeControlItem(this, QPointF(x + width, y + height), 3);
    ControlList << new CGShapeControlItem(this, QPointF(x, y + height), 4);
}

void CGShapeRectItem::SetRect(CGShapeRectangle rect)
{
    QPointF set = this->mapFromScene(rect.col, rect.row);
    ControlList[0]->SetPoint(set + QPointF(rect.width / 2, rect.height / 2));
    ControlList[1]->SetPoint(set);
    ControlList[2]->SetPoint(set + QPointF(rect.width, 0));
    ControlList[3]->SetPoint(set + QPointF(rect.width, rect.height));
    ControlList[4]->SetPoint(set + QPointF(0, rect.height));
    this->setFocus();
}

void CGShapeRectItem::GetRect(CGShapeRectangle &rect)
{
    QPointF GetRec = this->mapToScene(ControlList[1]->x(), ControlList[1]->y());
    rect.col = GetRec.x();
    rect.row = GetRec.y();
    rect.width = ControlList[2]->GetPoint().x() - ControlList[1]->GetPoint().x();
    rect.height = ControlList[3]->GetPoint().y() - ControlList[1]->GetPoint().y();
}

bool CGShapeRectItem::UpDate(int index)
{
    QPointF Pf = ControlList[index]->GetPoint();
    //角点分情况
    switch (index)
    {
    case 1:
        ControlList[2]->SetPoint(QPointF(ControlList[2]->GetPoint().x(), Pf.y()));
        ControlList[4]->SetPoint(QPointF(Pf.x(), ControlList[4]->GetPoint().y()));
        break;
    case 2:
        ControlList[1]->SetPoint(QPointF(ControlList[1]->GetPoint().x(), Pf.y()));
        ControlList[3]->SetPoint(QPointF(Pf.x(), ControlList[3]->GetPoint().y()));
        break;
    case 3:
        ControlList[2]->SetPoint(QPointF(Pf.x(), ControlList[2]->GetPoint().y()));
        ControlList[4]->SetPoint(QPointF(ControlList[4]->GetPoint().x(), Pf.y()));
        break;
    case 4:
        ControlList[1]->SetPoint(QPointF(Pf.x(), ControlList[1]->GetPoint().y()));
        ControlList[3]->SetPoint(QPointF(ControlList[3]->GetPoint().x(), Pf.y()));
        break;
    default:
        break;
    }
    //中心点
    ControlList[0]->SetPoint(QPointF((ControlList[1]->GetPoint().x() + ControlList[2]->GetPoint().x()) / 2, (ControlList[2]->GetPoint().y() + ControlList[3]->GetPoint().y()) / 2));
    return true;
}

QRectF CGShapeRectItem::boundingRect() const
{
    return  QRectF(ControlList[1]->GetPoint(), ControlList[3]->GetPoint());
}

void CGShapeRectItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    CGShapeBaseItem::paint(painter, option, widget);
    QRectF ret(ControlList[1]->GetPoint(), ControlList[3]->GetPoint());
    painter->drawRect(ret);
}
