#ifndef CGGRAPHICSLINEITEM_H
#define CGGRAPHICSLINEITEM_H

#include <QGraphicsLineItem>
#include <QGraphicsSceneWheelEvent>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneHoverEvent>
#include <CGImage2DGraphicsItemAdapter.h>

class CGGraphicsLineItem: public QGraphicsLineItem
{
    //Q_OBJECT

public:
    explicit CGGraphicsLineItem();

protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr);
    void wheelEvent(QGraphicsSceneWheelEvent *event);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event);
    void hoverMoveEvent(QGraphicsSceneHoverEvent *event);

private:
    QLineF  m_Line;
    QPointF m_PosPointF;
    QPointF m_LastPointF;
    QPointF m_CurrentPointF;

    bool bPressHit = false;
    int  iPointHit = 0;

public:
    int m_HitRange = 80;
};

#endif // CGGRAPHICSLINEITEM_H
