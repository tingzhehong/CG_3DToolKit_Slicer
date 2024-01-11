#ifndef CGSHAPECIRCLEITEM_H
#define CGSHAPECIRCLEITEM_H

#include <QObject>
#include <CGShapeBaseItem.h>

class CGShapeCircleItem : public CGShapeBaseItem
{
    Q_OBJECT

public:
    explicit CGShapeCircleItem(qreal x, qreal y, qreal R);

    void GetCircle(CGShapeCircle &Cir);

protected:
    virtual QRectF boundingRect() const override;
    virtual bool UpDate(int index) override;
    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    qreal Radius;
};

#endif // CGSHAPECIRCLEITEM_H
