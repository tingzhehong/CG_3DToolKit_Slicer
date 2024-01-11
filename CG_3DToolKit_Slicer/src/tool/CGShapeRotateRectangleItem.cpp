#include "CGShapeRotateRectangleItem.h"
#include "corecrt_math_defines.h"

CGShapeRotateRectangleItem::CGShapeRotateRectangleItem(qreal x, qreal y, qreal Lenth1, qreal Lenth2, qreal Pi) : CGShapeBaseItem(QPointF(x, y), RotateRectangle)
{
    angle = Pi;
    lenth1 = Lenth1 / 2;
    lenth2 = Lenth2 / 2;
    qreal s = sin(-angle);
    qreal c = cos(-angle);
    Pa1 = center + QPointF(0 * c - lenth2 * s, 0 * s + lenth2 * c);
    Pa2 = center + QPointF(lenth1 * c - 0 * s, lenth1 * s + 0 * c);
    Pa3 = center + QPointF(0 * c + lenth2 * s, 0 * s - lenth2 * c);
    Pa4 = center + QPointF(-lenth1 * c - 0 * s, -lenth1 * s + 0 * c);
    Parrow = center + QPointF((lenth1 + 100) * c - 0 * s, (lenth1 + 100) * s + 0 * c);
    //中心
    ControlList << new CGShapeControlItem(this, center, 0);
    //中心线控制点
    ControlList << new CGShapeControlItem(this, Pa1, 1);
    ControlList << new CGShapeControlItem(this, Pa2, 2);
    ControlList << new CGShapeControlItem(this, Pa3, 3);
    ControlList << new CGShapeControlItem(this, Pa4, 4);
}

void CGShapeRotateRectangleItem::GetRotateRect(CGShapeRotateRectangle &RRect)
{
    QPointF GetRRect = this->mapToScene(ControlList[0]->x(), ControlList[0]->y());
    RRect.col = GetRRect.x();
    RRect.row = GetRRect.y();
    RRect.phi = angle;
    RRect.lenth1 = lenth1;
    RRect.lenth2 = lenth2;
}

QRectF CGShapeRotateRectangleItem::boundingRect() const
{
    qreal tmp = (lenth1 > lenth2 ? lenth2 : lenth1);
    return QRectF(center.x() - tmp, center.y() - tmp, tmp * 2, tmp * 2);
}

bool CGShapeRotateRectangleItem::UpDate(int index)
{
    QPointF Pf = ControlList[index]->GetPoint();
    qreal dx = Pf.x() - center.x();
    qreal dy = Pf.y() - center.y();
    if (dx >= 0 && dy < 0)
    {
        angle = atan2((-1) * (dy), dx);
    }
    else if (dx < 0 && dy < 0)
    {
        angle = atan2((-1) * dy, dx);
    }
    else if (dx < 0 && dy >= 0)
    {
        angle = M_PI * 2 + atan2((-1) * dy, dx);
    }
    else if (dx >= 0 && dy >= 0)
    {
        angle = M_PI * 2 - atan2(dy, dx);
    }
    //角度补偿 角度方向逆时针 控制点排列方向 顺时针
    switch (index)
    {
    case 1:
        angle += M_PI / 2;
        break;
    case 3:
        angle += M_PI * 3 / 2;
        break;
    case 4:
        angle += M_PI;
        break;
    }
    qreal s = sin(-angle);
    qreal c = cos(-angle);
    if (index == 2 || index == 4)
    {
        lenth1 = sqrt(dx * dx + dy * dy);
    }
    else if (index == 1 || index == 3)
    {
        lenth2 = sqrt(dx * dx + dy * dy);
    }
    Pa1 = center + QPointF(0 * c - lenth2 * s, 0 * s + lenth2 * c);
    Pa2 = center + QPointF(lenth1 * c - 0 * s, lenth1 * s + 0 * c);
    Pa3 = center + QPointF(0 * c + lenth2 * s, 0 * s - lenth2 * c);
    Pa4 = center + QPointF(-lenth1 * c - 0 * s, -lenth1 * s + 0 * c);
    Parrow = center + QPointF((lenth1 + 100) * c - 0 * s, (lenth1 + 100) * s + 0 * c);
    ControlList[1]->SetPoint(Pa1);
    ControlList[2]->SetPoint(Pa2);
    ControlList[3]->SetPoint(Pa3);
    ControlList[4]->SetPoint(Pa4);

    return true;
}

void CGShapeRotateRectangleItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);
    CGShapeBaseItem::paint(painter, option, widget);
    //画角度方向线
    painter->drawLine(Parrow, Pa2);
    //绘制方向箭头
    float l = 60.0;//箭头长度
    double atn1 = atan2((Parrow.y() - Pa2.y()), (Parrow.x() - Pa2.x()));
    double atn2 = atan2((Parrow.x() - Pa2.x()), (Parrow.y() - Pa2.y()));
    QPointF Arrow1(Parrow.x() - l * cos(atn1 - 0.5), Parrow.y() - l * sin(atn1 - 0.5));
    QPointF Arrow2(Parrow.x() - l * sin(atn2 - 0.5), Parrow.y() - l * cos(atn2 - 0.5));
    painter->drawLine(Parrow, Arrow1);
    painter->drawLine(Parrow, Arrow2);
    //绘制旋转矩形
    painter->save();
    painter->translate(center);
    painter->rotate(-angle * 180 / M_PI);
    painter->drawRect(QRectF(-lenth1, -lenth2, lenth1 * 2, lenth2 * 2));
    painter->restore();
}
