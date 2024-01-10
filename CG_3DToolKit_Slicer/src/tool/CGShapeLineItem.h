#ifndef CGSHAPELINEITEM_H
#define CGSHAPELINEITEM_H

#include <QObject>
#include <CGShapeBaseItem.h>

class CGShapeLineItem : public CGShapeBaseItem
{
    Q_OBJECT

public:
    explicit CGShapeLineItem(qreal x1, qreal y1, qreal x2, qreal y2);

    void SetLine(CGShapeLine line);
    void GetLine(CGShapeLine &line);

protected:
    virtual bool UpDate(int index) override;
    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    QPointF P1;
    QPointF P2;
};

#endif // CGSHAPELINEITEM_H
