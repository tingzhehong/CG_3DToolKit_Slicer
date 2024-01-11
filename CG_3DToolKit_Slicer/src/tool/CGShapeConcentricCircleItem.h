#ifndef CGSHAPECONCENTRICCIRCLEITEM_H
#define CGSHAPECONCENTRICCIRCLEITEM_H

#include <QObject>
#include <CGShapeBaseItem.h>

class CGShapeConcentricCircleItem : public CGShapeBaseItem
{
    Q_OBJECT

public:
    explicit CGShapeConcentricCircleItem(qreal x, qreal y, qreal RadiusMin, qreal RadiusMax);

    void GetConcentricCircle(CGShapeConcentricCircle &CCir);

    int segment_line_num = 0;
    std::vector<QPointF> ring_small_points = std::vector<QPointF>(1000);
    std::vector<QPointF> ring_big_points = std::vector<QPointF>(1000);

protected:
    virtual QRectF boundingRect() const override;
    virtual bool UpDate(int index) override;
    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    qreal RadiusMin;
    qreal RadiusMax;
};

#endif // CGSHAPECONCENTRICCIRCLEITEM_H
