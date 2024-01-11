#ifndef CGSHAPEROTATERECTANGLEITEM_H
#define CGSHAPEROTATERECTANGLEITEM_H

#include <QObject>
#include <CGShapeBaseItem.h>

class CGShapeRotateRectangleItem : public CGShapeBaseItem
{
    Q_OBJECT

public:
    explicit CGShapeRotateRectangleItem(qreal x, qreal y, qreal Lenth1, qreal Lenth2, qreal Pi);

    void GetRotateRect(CGShapeRotateRectangle &RRect);

protected:
    virtual QRectF boundingRect() const override;
    virtual bool UpDate(int index) override;
    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

protected:
    qreal angle = 0;
    qreal lenth1 = 0;
    qreal lenth2 = 0;
    QPointF Pa1;
    QPointF Pa2;
    QPointF Pa3;
    QPointF Pa4;
    QPointF Parrow;
};

#endif // CGSHAPEROTATERECTANGLEITEM_H
