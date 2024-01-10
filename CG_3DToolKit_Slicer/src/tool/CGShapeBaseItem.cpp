#include "CGShapeBaseItem.h"
#include <QDebug>

qreal CGShapeBaseItem::ControlSize = 30;

CGShapeBaseItem::CGShapeBaseItem(QPointF center, ItemType type) :center(center), types(type)
{
    noSelected.setColor(QColor(0, 0, 255));
    noSelected.setWidth(LineWidth);
    noSelected.setCosmetic(true);
    isSelected.setColor(QColor(0, 255, 0));
    isSelected.setWidth(LineWidth);
    isSelected.setCosmetic(true);

    Pen = noSelected;

    fillColor = QColor(0, 160, 230, 50);

    setHandlesChildEvents(false);  //设置后才能将事件传递到子元素
    setFlags(QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsFocusable);
}

void CGShapeBaseItem::SetIndex(int num)
{
    if (ControlList.length() > 0)
    {
       ControlList[0]->index = num;
    }
}

void CGShapeBaseItem::SetScale(double value)
{
    scaler = value;
}

void CGShapeBaseItem::focusInEvent(QFocusEvent *event)
{
    Q_UNUSED(event);
    Pen = isSelected;
    for (int i = 1; i < ControlList.length(); i++)
    {
        ControlList[i]->setVisible(true);
    }
}

void CGShapeBaseItem::focusOutEvent(QFocusEvent *event)
{
    Q_UNUSED(event);
    Pen = noSelected;
    for (int i = 1; i < ControlList.length(); i++)
    {
        ControlList[i]->setVisible(false);
    }
}

void CGShapeBaseItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    //缩放控制点尺寸
    for (int i = 0; i < ControlList.size(); i++)
    {
        ControlList[i]->setScale(1 / (*scale));
    }
    painter->save();
    painter->setBrush(Qt::NoBrush);

    //字体大小
    QFont font;
    font.setPointSizeF(15 / (*scale));
    painter->setFont(font);
    painter->setPen(Qt::black);
    painter->drawText(ControlList[0]->GetPoint() + QPointF(-ControlSize / (*scale), -(ControlSize + 3) / (*scale)), ItemDiscribe);
    painter->restore();

    //子类绘制时状态
    LineWidth = 2 / (*scale);
    Pen.setWidthF(LineWidth);
    painter->setPen(Pen);
}
