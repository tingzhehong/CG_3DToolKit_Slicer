#ifndef CGSHAPERECTITEM_H
#define CGSHAPERECTITEM_H

#include <QObject>
#include <CGShapeBaseItem.h>

class CGShapeRectItem : public CGShapeBaseItem
{
    Q_OBJECT

public:
    explicit CGShapeRectItem(qreal x, qreal y, qreal width, qreal height);

    void SetRect(CGShapeRectangle rect);
    void GetRect(CGShapeRectangle &rect);

protected:
    virtual bool UpDate(int index) override;
    virtual QRectF boundingRect() const override;
    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;
};

#endif // CGSHAPERECTITEM_H
