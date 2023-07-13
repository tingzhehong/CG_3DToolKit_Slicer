#ifndef NODEITEM_H
#define NODEITEM_H

#include <QGraphicsItem>
#include <QFont>
#include <QBrush>
#include <QUuid>
#include "PortItem.h"

class NodeItem : public QGraphicsObject
{
    Q_OBJECT

public:
    explicit NodeItem(QWidget *widget, QGraphicsItem *parent = Q_NULLPTR);

private:

    QWidget *m_widget;
    QUuid m_uuid;
    QSizeF m_size;
    QSizeF m_indentSize;
    QSizeF m_titleSize;
    QFont m_font;
    QColor m_fontColor;
    QColor m_titleColor;
    QColor m_backgroundColor;
    QBrush m_backgroundBrush;
    bool m_isUserBrush;
    bool m_isSelected;

public:
    QList<PortItem*> m_portList;
    QString m_title;

private:
    QRectF boundingRect() const;
    QPainterPath shape() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *item, QWidget *widget);

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

signals:
    void selected(bool state);
    void positionChanged();

public slots:
    bool isHovered(const QPointF &point);
    PortItem *isHoveredPort(const QPointF &point);

    void updateSize();

    QWidget *widget(){return m_widget;}

    void addPortIn(PortItem *port);
    PortItem *createPortIn(int posY, QColor color);
    PortItem *createPortIn(int posY, QColor color, uint num);

    void addPortOut(PortItem *port);
    PortItem *createPortOut(int posY, QColor color);
    PortItem *createPortOut(int posY, QColor color, uint num);

    QList<PortItem*> portList(){return m_portList;}
    PortItem* portAt(uint num);

    void removePortAt(uint num);
    void removePort(PortItem *port);

    QUuid uuid(){return m_uuid;}
    void setUuid(const QUuid &uuid){m_uuid = uuid;}

    QString title(){return m_title;}
    void setTitle(const QString &title);
    void clearTitle();

    qreal width(){return m_size.width();}
    qreal height(){return m_size.height();}
    QSizeF size(){return m_size;}

    void setFont(const QFont &font);
    void setFontColor(const QColor &color);
    void setTitleColor(const QColor &color);
    void setBackgroundColor(const QColor &color);
    void setBackgroundBrush(const QBrush &brush);


public:
    unsigned int NodeID(){return m_NodeID;}
    void  setNodeID(const unsigned int id){m_NodeID = id;}

    QString NodeName(){return m_NodeName;}
    void setNodeName(const QString name){m_NodeName = name;}

    void PortClass();

public:
    QList<PortItem*> m_InPortItem;
    QList<PortItem*> m_OutPortItem;

    NodeItem *m_Prev, *m_Next;

    bool m_GroupNode;
    bool m_AlgorithmNode;
    unsigned int m_NodeID;
    QString m_NodeName;
    QVector<QVariant> m_Parameters;

};

#endif // NODEITEM_H
