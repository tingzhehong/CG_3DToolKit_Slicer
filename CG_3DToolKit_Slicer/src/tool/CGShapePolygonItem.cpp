#include "CGShapePolygonItem.h"

CGShapePolygonItem::CGShapePolygonItem() : CGShapeBaseItem(QPointF(0, 0), Polygon)
{
    init_points.clear();
    list_ps.clear();
    ControlList << new CGShapeControlItem(this, center, 0);
    ControlList[0]->setVisible(false);
    Finished = false;
}

QPointF CGShapePolygonItem::getCentroid(QList<QPointF> list)
{
    qreal x = 0;
    qreal y = 0;
    for (auto& temp : list)
    {
        x += temp.x();
        y += temp.y();
    }
    x = x / list.size();
    y = y / list.size();
    return QPointF(x, y);
}

void CGShapePolygonItem::getMaxLength()
{
    QVector<qreal> vec;
    vec.reserve(200);
    vec.clear();
    for (int i = 1; i < ControlList.length(); i++)
    {
        qreal dis = sqrt(pow(center.x() - ControlList[i]->x(), 2) + pow(center.y() - ControlList[i]->y(), 2));
        vec.append(dis);
    }
    qreal ret = 0;
    for (auto& temp : vec)
    {
        if (temp > ret)
        {
            ret = temp;
        }
    }
    Radius = ret;
}

void CGShapePolygonItem::GetPolygon(CGShapePolygon &polygon)
{
    QList<QPointF> list_p;
    list_p.reserve(200);
    list_p.clear();
    for (int i = 1; i < ControlList.length(); i++)
    {
        list_p.append(mapToScene(ControlList[i]->GetPoint()));
    }
    polygon.list_p = list_p;
    polygon.points = init_points;
    polygon.list_ps = list_ps;
}

void CGShapePolygonItem::pushPoint(QPointF p, QList<QPointF> list, bool isCenter)
{
    if (!Finished)
    {
        center = getCentroid(list);
        getMaxLength();
        if (isCenter)
        {
            ControlList[0]->SetPoint(center);
            ControlList[0]->setVisible(true);
            Finished = true;
        }
        else
        {
            auto tmp = new CGShapeControlItem(this, p, ControlList.length());
            tmp->setVisible(true);
            ControlList << tmp;
            init_points.append(p);
            list_ps.append(list);
        }
        this->update();
    }
}

QRectF CGShapePolygonItem::boundingRect() const
{
    return QRectF(center.x() - Radius, center.y() - Radius, Radius * 2, Radius * 2);
}

bool CGShapePolygonItem::UpDate(int index)
{
    Q_UNUSED(index);
    QList<QPointF> list;
    list.reserve(200);
    list.clear();
    for (int i = 1; i < ControlList.length(); i++)
    {
        list << ControlList[i]->GetPoint();
    }
    center = getCentroid(list);
    ControlList[0]->SetPoint(center);
    return true;
}

void CGShapePolygonItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);
    Pen.setWidthF(LineWidth);
    painter->setPen(Pen);

    if (Finished)
    {
        for (int i = 1; i < ControlList.length() - 1; i++)
        {
            painter->drawLine(ControlList[i]->GetPoint(), ControlList[i + 1]->GetPoint());
        }
        painter->drawLine(ControlList[ControlList.length() - 1]->GetPoint(), ControlList[1]->GetPoint());
    }
    else
    {
        for (int i = 1; i < ControlList.length() - 1; i++)
        {
            painter->drawLine(ControlList[i]->GetPoint(), ControlList[i + 1]->GetPoint());
        }
    }
}
