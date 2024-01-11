#ifndef CGGRAPHICSRECTITEM_H
#define CGGRAPHICSRECTITEM_H

#include <QGraphicsRectItem>
#include <QGraphicsSceneWheelEvent>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneHoverEvent>
#include <CGImage2DGraphicsItemAdapter.h>

class CGGraphicsRectItem: public QGraphicsRectItem
{
    //Q_OBJECT

public:
    explicit CGGraphicsRectItem();

protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr);
    void wheelEvent(QGraphicsSceneWheelEvent *event);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event);
    void hoverMoveEvent(QGraphicsSceneHoverEvent *event);

private:
    QRectF  m_Rect;
    QPointF m_PosPointF;
    QPointF m_LastPointF;
    QPointF m_CurrentPointF;

    bool bPressHit = false;
    int  iPointHit = 0;

public:
    int m_HitRange = 60;
};

#endif // CGGRAPHICSRECTITEM_H
