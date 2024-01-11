#ifndef CGSHAPEPOLYGONITEM_H
#define CGSHAPEPOLYGONITEM_H

#include <QObject>
#include <CGShapeBaseItem.h>

class CGShapePolygonItem : public CGShapeBaseItem
{
    Q_OBJECT

public:
    explicit CGShapePolygonItem();

    QPointF getCentroid(QList<QPointF> list);
    void getMaxLength();
    void GetPolygon(CGShapePolygon &polygon);

public slots:
    void pushPoint(QPointF p, QList<QPointF> list, bool isCenter);

protected:
    virtual QRectF boundingRect() const override;
    virtual bool UpDate(int index) override;
    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

protected:
    qreal Radius;
    bool Finished;
    QList<QPointF> init_points;
    QList<QList<QPointF>> list_ps;
};

#endif // CGSHAPEPOLYGONITEM_H
