#ifndef CGSHAPECONTROLITEM_H
#define CGSHAPECONTROLITEM_H

#pragma once
#include <QObject>
#include <QAbstractGraphicsShapeItem>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsScene>
#include <QKeyEvent>
#include <QPainter>
#include <QPen>

class CGShapeControlItem : public QObject, public QAbstractGraphicsShapeItem
{
     Q_OBJECT

public:
    explicit CGShapeControlItem(QGraphicsItemGroup* parent, QPointF p, int type);

    void SetPoint(QPointF p){m_Point = p; update();}
    QPointF GetPoint(){return m_Point;}
    QPointF GetLastPoint(){return m_LastPoint;}

    qreal dX(){return dx;}
    qreal dY(){return dy;}

    int index = 0;

protected:
    virtual QRectF boundingRect() const override;
    virtual void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget *widget) override;
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) override;

private:
    QPen m_Pen = pen();
    QPointF m_Point;
    int m_PointType;
    QRectF m_Rect;
    qreal dx;
    qreal dy;
    QPointF m_LastPoint;
};

#endif // CGSHAPECONTROLITEM_H
