#ifndef CGSHAPEBASEITEM_H
#define CGSHAPEBASEITEM_H

#pragma once
#include <QObject>
#include <CGShapeControlItem.h>


///直线
struct CGShapeLine
{
    CGShapeLine()
    {
    }

    CGShapeLine(float x1, float y1, float x2, float y2)
    {
        p1_x = x1;
        p1_y = y1;
        p2_x = x2;
        p2_y = y2;
    }
    float p1_x;
    float p1_y;
    float p2_x;
    float p2_y;
};

///矩形
struct CGShapeRectangle
{
    CGShapeRectangle()
    {
    }

    CGShapeRectangle(float x, float y, float Width, float Height)
    {
        row = y;
        col = x;
        width = Width;
        height = Height;
    }
    float row;
    float col;
    float width;
    float height;
};

///旋转矩形
struct CGShapeRotatedRect
{
    CGShapeRotatedRect()
    {
    }

    CGShapeRotatedRect(float x, float y, float Phi, float Lenth1, float Lenth2)
    {
        row = y;
        col = x;
        phi = Phi;
        lenth1 = Lenth1;
        lenth2 = Lenth2;
    }
    float row;
    float col;
    float phi;
    float lenth1;
    float lenth2;
};

///圆
struct CGShapeCircle
{
    CGShapeCircle()
    {
    }

    CGShapeCircle(float x, float y, float Radius)
    {
        row = y;
        col = x;
        radius = Radius;
    }
    float row;
    float col;
    float radius;
};

///同心圆
struct CGShapeConcentricCircle
{
    CGShapeConcentricCircle()
    {
    }

    CGShapeConcentricCircle(float x, float y, float RadiusMin, float RadiusMax)
    {
        row = y;
        col = x;
        small_radius = RadiusMin;
        big_radius = RadiusMax;
    }
    float row;
    float col;
    float small_radius;
    float big_radius;
};

///多边形
struct CGShapePolygon
{
    CGShapePolygon()
    {
    }

    CGShapePolygon(QList<QPointF> Points, QList<QPointF> List_P, QList<QList<QPointF>> List_Ps)
    {
        points = Points;
        list_p = List_P;
        list_ps = List_Ps;
    }
    QList<QPointF> points;
    QList<QPointF> list_p;
    QList<QList<QPointF>> list_ps;
};


class CGShapeBaseItem : public QObject, public QGraphicsItemGroup
{
    Q_OBJECT

public:
    enum ItemType
    {
        Line,                // 直线
        Rectangle,           // 矩形
        RotateRectangle,     // 旋转矩形
        Circle,              // 圆
        ConcentricCircle,    // 同心圆
        Polygon,             // 多边形
    };

public:
    explicit CGShapeBaseItem(QPointF center, ItemType type);

public:
    virtual bool UpDate(int index) = 0;

    QList<CGShapeControlItem* > ControlList;
    qreal scaler = 1;
    qreal *scale = &scaler;

    void SetIndex(int num);
    void SetScale(double value);
    static qreal ControlSize;

protected:
    virtual void focusInEvent(QFocusEvent* event) override;
    virtual void focusOutEvent(QFocusEvent* event) override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

protected:
    QPointF center;
    ItemType types;
    QPen isSelected;
    QPen noSelected;
    QColor fillColor;
    QPen Pen;
    qreal LineWidth = 3;  //控制点初始尺寸
    QString ItemDiscribe = QString::fromLocal8Bit("");
};

#endif // CGSHAPEBASEITEM_H
