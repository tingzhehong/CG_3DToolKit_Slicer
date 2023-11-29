#include "ValueTerminalNodeBlock.h"
#include <QLabel>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonArray>
#include <QDebug>

ValueTerminalNodeBlock::ValueTerminalNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    NodeItemFactory(tr(u8"数值终端"), 1, 0);
}

void ValueTerminalNodeBlock::Run()
{
    m_NodeItem->PortClass();

    QJsonObject json;
    QVariant var = m_NodeItem->m_InPortItem.at(0)->value();

    if (var.canConvert<QJsonObject>())
        json = var.toJsonObject();
    else qDebug() << "Can't convert to json object!";

    QString QJsonValueText = NULL;
    QJsonValueText.append("Value List").append(" : ").append(QString::number(json.count())).append("\n");
    QJsonValueText.append("{").append("\n");

    QStringList keys = json.keys();
    for (int i = 0; i < keys.count(); ++i)
    {
        QString key = keys.at(i);
        QString value = NULL;
        QJsonValue jsonvalue = json.value(keys.at(i));

        if (jsonvalue.isDouble())
            value = QString::number(jsonvalue.toDouble(), 'f', 6);
        else
            value = jsonvalue.toString();

        QJsonValueText.append("    ").append(key).append(" : ").append(value).append("\n");
    }
    QJsonValueText.append("}").append("\n");

    m_NodeItem->m_Parameters[u8"Json"] = QJsonValueText;
    //qDebug() << "JsonValueText: "<< QJsonValueText;

    emit SignalShowValue();
    m_IsRuned = true;
}

NodeItem *ValueTerminalNodeBlock::CreatNodeItem10(const QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *label = new QLabel(tr(u8"算子: ") + nodename, widget);
    label->resize(100, 20);
    label->move(50, 15);

    QLabel *labelId = new QLabel("ID: " + QString::number(m_IDCounter), widget);
    labelId->resize(60, 20);
    labelId->move(2, 15);

    m_NodeItem = m_NodeView->createNode(widget);
    m_NodeItem->setTitle(nodename);
    m_NodeItem->setNodeName(nodename);
    m_NodeItem->setNodeID(m_IDCounter);

    int x = RandPos();
    int y = RandPos();
    m_NodeItem->setPos(x, y);

    m_NodeItem->createPortIn(8, QColor(Qt::green));

    return m_NodeItem;
}
