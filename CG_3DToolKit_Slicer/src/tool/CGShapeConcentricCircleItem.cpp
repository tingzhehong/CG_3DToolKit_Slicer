#include "CGShapeConcentricCircleItem.h"
#include "corecrt_math_defines.h"

CGShapeConcentricCircleItem::CGShapeConcentricCircleItem(qreal x, qreal y, qreal radiusMin, qreal radiusMax) : CGShapeBaseItem(QPointF(x, y), ConcentricCircle)
{
    RadiusMax = radiusMax;
    RadiusMin = radiusMin > radiusMax ? radiusMax : radiusMin;
    ControlList << new CGShapeControlItem(this, center, 0);
    ControlList << new CGShapeControlItem(this, center + QPointF(RadiusMin, 0), 1);
    ControlList << new CGShapeControlItem(this, center + QPointF(RadiusMax, 0), 2);
}

void CGShapeConcentricCircleItem::GetConcentricCircle(CGShapeConcentricCircle &CCir)
{
    QPointF GetCCircle = this->mapToScene(ControlList[0]->x(), ControlList[0]->y());
    CCir.col = GetCCircle.x();
    CCir.row = GetCCircle.y();
    CCir.small_radius = RadiusMin;
    CCir.big_radius = RadiusMax;
}

QRectF CGShapeConcentricCircleItem::boundingRect() const
{
    return QRectF(center.x() - RadiusMax, center.y() - RadiusMax, RadiusMax * 2, RadiusMax * 2);
}

bool CGShapeConcentricCircleItem::UpDate(int index)
{
    QPointF Pf = ControlList[index]->GetPoint();
    QPointF tmp = Pf - center;
    qreal R = sqrt(tmp.x() * tmp.x() + tmp.y() * tmp.y());
    if (index == 1)
    {
        if (R > RadiusMax)
            return false;
        RadiusMin = R;
    }
    else if (index == 2)
    {
        if (R < RadiusMin)
            return false;
        RadiusMax = R;
    }
    return true;
}

void CGShapeConcentricCircleItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    CGShapeBaseItem::paint(painter, option, widget);
    QPainterPath pth;
    pth.addEllipse(QRectF(center.x() - RadiusMin, center.y() - RadiusMin, RadiusMin * 2, RadiusMin * 2));
    pth.addEllipse(QRectF(center.x() - RadiusMax, center.y() - RadiusMax, RadiusMax * 2, RadiusMax * 2));
    painter->setBrush(QBrush(fillColor));
    painter->drawPath(pth);

    //绘制分割线
    painter->setPen(QPen(QColor(10, 255, 255, 255), 1));
    QLine line;
    double angle = 0;
    ring_small_points.clear();
    ring_big_points.clear();
    for (int i = 0; i <= segment_line_num; i++)
    {
        QPointF pf_min, pf_max;
        if (0 <= angle && angle < 90)
        {
            line = QLine(QPoint(center.x() + cos(angle * M_PI / 180) * RadiusMin, center.y() - sin(angle * M_PI / 180) * RadiusMin), QPoint(center.x() + cos(angle * M_PI / 180) * RadiusMax, center.y() - sin(angle * M_PI / 180) * RadiusMax));
            pf_min = this->mapToScene(QPointF(center.x() + cos(angle * M_PI / 180) * RadiusMin, center.y() - sin(angle * M_PI / 180) * RadiusMin));
            ring_small_points.push_back(pf_min);
            pf_max = this->mapToScene(QPointF(center.x() + cos(angle * M_PI / 180) * RadiusMax, center.y() - sin(angle * M_PI / 180) * RadiusMax));
            ring_big_points.push_back(pf_max);
        }
        else if (90 <= angle && angle < 180)
        {
            line = QLine(QPoint(center.x() - sin((angle - 90) * M_PI / 180) * RadiusMin, center.y() - cos((angle - 90) * M_PI / 180) * RadiusMin), QPoint(center.x() - sin((angle - 90) * M_PI / 180) * RadiusMax, center.y() - cos((angle - 90) * M_PI / 180) * RadiusMax));
            pf_min = this->mapToScene(QPointF(center.x() - sin((angle - 90) * M_PI / 180) * RadiusMin, center.y() - cos((angle - 90) * M_PI / 180) * RadiusMin));
            ring_small_points.push_back(pf_min);
            pf_max = this->mapToScene(QPointF(center.x() - sin((angle - 90) * M_PI / 180) * RadiusMax, center.y() - cos((angle - 90) * M_PI / 180) * RadiusMax));
            ring_big_points.push_back(pf_max);
        }
        else if (180 <= angle && angle < 270)
        {
            line = QLine(QPoint(center.x() - cos((angle - 180) * M_PI / 180) * RadiusMin, center.y() + sin((angle - 180) * M_PI / 180) * RadiusMin), QPoint(center.x() - cos((angle - 180) * M_PI / 180) * RadiusMax, center.y() + sin((angle - 180) * M_PI / 180) * RadiusMax));
            pf_min = this->mapToScene(QPointF(center.x() - cos((angle - 180) * M_PI / 180) * RadiusMin, center.y() + sin((angle - 180) * M_PI / 180) * RadiusMin));
            ring_small_points.push_back(pf_min);
            pf_max = this->mapToScene(QPointF(center.x() - cos((angle - 180) * M_PI / 180) * RadiusMax, center.y() + sin((angle - 180) * M_PI / 180) * RadiusMax));
            ring_big_points.push_back(pf_max);
        }
        else if (270 <= angle && angle < 360)
        {
            line = QLine(QPoint(center.x() + sin((angle - 270) * M_PI / 180) * RadiusMin, center.y() + cos((angle - 270) * M_PI / 180) * RadiusMin), QPoint(center.x() + sin((angle - 270) * M_PI / 180) * RadiusMax, center.y() + cos((angle - 270) * M_PI / 180) * RadiusMax));
            pf_min = this->mapToScene(QPointF(center.x() + sin((angle - 270) * M_PI / 180) * RadiusMin, center.y() + cos((angle - 270) * M_PI / 180) * RadiusMin));
            ring_small_points.push_back(pf_min);
            pf_max = this->mapToScene(QPointF(center.x() + sin((angle - 270) * M_PI / 180) * RadiusMax, center.y() + cos((angle - 270) * M_PI / 180) * RadiusMax));
            ring_big_points.push_back(pf_max);
        }
        painter->drawLine(line);
        angle = angle + (360.0 / (double)segment_line_num);
    }
}
