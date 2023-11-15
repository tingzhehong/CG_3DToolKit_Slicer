#include "NodeBlock.h"
#include <QLabel>
#include <QLineEdit>
#include <QDebug>

NodeBlock::NodeBlock(NodeView *nodeview, QWidget *parent) : QWidget(parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;
    m_IDCounterMinus = nodeview->m_IDCounterMinus;
}

NodeItem *NodeBlock::NodeItemFactory(QString nodename, int in, int out)
{
    int index = in * 10 + out;

    switch (index)
    {
    case 00:
        m_NodeItem = CreatNodeItem00(nodename);
        ++m_IDCounter;
        break;
    case 01:
        m_NodeItem = CreatNodeItem01(nodename);
        ++m_IDCounter;
        break;
    case 10:
        m_NodeItem = CreatNodeItem10(nodename);
        ++m_IDCounter;
        break;
    case 11:
        m_NodeItem = CreatNodeItem11(nodename);
        ++m_IDCounter;
        break;
    case 21:
        m_NodeItem = CreatNodeItem21(nodename);
        ++m_IDCounter;
        break;
    case 31:
        m_NodeItem = CreatNodeItem31(nodename);
        ++m_IDCounter;
        break;
    case 41:
        m_NodeItem = CreatNodeItem41(nodename);
        ++m_IDCounter;
        break;
    case 12:
        m_NodeItem = CreatNodeItem12(nodename);
        ++m_IDCounter;
        break;
    case 22:
        m_NodeItem = CreatNodeItem22(nodename);
        ++m_IDCounter;
        break;
    case 32:
        m_NodeItem = CreatNodeItem32(nodename);
        ++m_IDCounter;
        break;
    case 33:
        m_NodeItem = CreatNodeItem33(nodename);
        ++m_IDCounter;
        break;
    default:
        m_NodeItem = CreatNodeItemIO(nodename, in, out);
        break;
    }

    return m_NodeItem;
}

NodeItem *NodeBlock::NodeItemNumberInput(QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *label = new QLabel(tr(u8"数值:"), widget);
    label->resize(60, 20);
    label->move(2, 15);

    QLineEdit *lineEdit = new QLineEdit("", widget);
    lineEdit->resize(105, 20);
    lineEdit->move(40, 15);

    m_NodeItem = m_NodeView->createNode(widget);
    m_NodeItem->setTitle(nodename);
    m_NodeItem->setNodeName(nodename);
    m_NodeItem->setNodeID(m_IDCounterMinus);

    int x = RandPos();
    int y = RandPos();
    m_NodeItem->setPos(x, y);

    PortItem *portOut = m_NodeItem->createPortOut(8, QColor(Qt::cyan));
    connect(lineEdit, &QLineEdit::textChanged, this, [=](QString str) {
            portOut->setValue(QVariant::fromValue(str.trimmed().toFloat()));
            m_NodeItem->m_Parameters[u8"值"] = portOut->value();
            m_IsRuned = true;
    });
    connect(m_NodeItem, &NodeItem::parametersChanged, this, [=] {
            float value = m_NodeItem->m_Parameters.value(u8"值").toFloat();
            float val = (int)value;
            if (value != val)
                lineEdit->setText(QString::asprintf("%.6f", value));
            else
                lineEdit->setText(m_NodeItem->m_Parameters.value(u8"值").toString());
    });

    return m_NodeItem;
}

NodeItem *NodeBlock::NodeItemNumberOutput(QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *label = new QLabel(tr(u8"数值:"), widget);
    label->resize(60, 20);
    label->move(2, 15);

    QLineEdit *lineEdit = new QLineEdit("", widget);
    lineEdit->resize(105, 20);
    lineEdit->move(40, 15);
    lineEdit->setReadOnly(true);

    m_NodeItem = m_NodeView->createNode(widget);
    m_NodeItem->setTitle(nodename);
    m_NodeItem->setNodeName(nodename);
    m_NodeItem->setNodeID(m_IDCounterMinus);

    int x = RandPos();
    int y = RandPos();
    m_NodeItem->setPos(x, y);

    PortItem *portIn = m_NodeItem->createPortIn(8, QColor(Qt::cyan));
    connect(portIn, &PortItem::valueChanged, this, [=](QVariant value){ lineEdit->setText(QString::asprintf("%.6f", value.toFloat())); });

    return m_NodeItem;
}

bool NodeBlock::IsValid()
{
    int num = m_NodeItem->m_portList.size();
    bool ret = true;
    for (int i = 0; i < num; ++i)
    {
        PortItem *port = m_NodeItem->m_portList.at(i);
        if (port->portType() == PortItem::TypeIn)
        {
            QVariant var = port->value();
            if (var.isNull())
            {
                ret = false;
                break;
            }
        }
    }
    return ret;
}

bool NodeBlock::IsRuned()
{
    return m_IsRuned;
}

NodeItem *NodeBlock::CreatNodeItem01(const QString nodename)
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

    m_NodeItem->createPortOut(8, QColor(Qt::cyan));

    return m_NodeItem;
}

NodeItem *NodeBlock::CreatNodeItem10(const QString nodename)
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

    m_NodeItem->createPortIn(8, QColor(Qt::cyan));

    return m_NodeItem;
}

NodeItem *NodeBlock::CreatNodeItem11(const QString nodename)
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

    m_NodeItem->createPortIn(8, QColor(Qt::cyan));
    m_NodeItem->createPortOut(8, QColor(Qt::cyan));

    return m_NodeItem;
}

NodeItem *NodeBlock::CreatNodeItem21(const QString nodename)
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

    m_NodeItem->createPortIn(8, QColor(Qt::cyan));
    m_NodeItem->createPortIn(24, QColor(Qt::cyan));
    m_NodeItem->createPortOut(8, QColor(Qt::cyan));

    return m_NodeItem;
}

NodeItem *NodeBlock::CreatNodeItem31(const QString nodename)
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

    m_NodeItem->createPortIn(8, QColor(Qt::cyan));
    m_NodeItem->createPortIn(24, QColor(Qt::cyan));
    m_NodeItem->createPortIn(40, QColor(Qt::cyan));
    m_NodeItem->createPortOut(8, QColor(Qt::cyan));

    return m_NodeItem;
}

NodeItem *NodeBlock::CreatNodeItem41(const QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 60);

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

    m_NodeItem->createPortIn(8, QColor(Qt::cyan));
    m_NodeItem->createPortIn(24, QColor(Qt::cyan));
    m_NodeItem->createPortIn(40, QColor(Qt::cyan));
    m_NodeItem->createPortIn(56, QColor(Qt::cyan));
    m_NodeItem->createPortOut(8, QColor(Qt::cyan));

    return m_NodeItem;
}

NodeItem *NodeBlock::CreatNodeItem12(const QString nodename)
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

    m_NodeItem->createPortIn(8, QColor(Qt::cyan));
    m_NodeItem->createPortOut(8, QColor(Qt::cyan));
    m_NodeItem->createPortOut(24, QColor(Qt::cyan));

    return m_NodeItem;
}

NodeItem *NodeBlock::CreatNodeItem22(const QString nodename)
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

    m_NodeItem->createPortIn(8, QColor(Qt::cyan));
    m_NodeItem->createPortIn(24, QColor(Qt::cyan));
    m_NodeItem->createPortOut(8, QColor(Qt::cyan));
    m_NodeItem->createPortOut(24, QColor(Qt::cyan));

    return m_NodeItem;
}

NodeItem *NodeBlock::CreatNodeItem32(const QString nodename)
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

    m_NodeItem->createPortIn(8, QColor(Qt::cyan));
    m_NodeItem->createPortIn(24, QColor(Qt::yellow));
    m_NodeItem->createPortIn(40, QColor(Qt::red));
    m_NodeItem->createPortOut(8, QColor(Qt::cyan));
    m_NodeItem->createPortOut(24, QColor(Qt::yellow));

    return m_NodeItem;
}

NodeItem *NodeBlock::CreatNodeItem33(const QString nodename)
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

    m_NodeItem->createPortIn(8, QColor(Qt::cyan));
    m_NodeItem->createPortIn(24, QColor(Qt::yellow));
    m_NodeItem->createPortIn(40, QColor(Qt::red));
    m_NodeItem->createPortOut(8, QColor(Qt::cyan));
    m_NodeItem->createPortOut(24, QColor(Qt::yellow));
    m_NodeItem->createPortOut(40, QColor(Qt::red));

    return m_NodeItem;
}

int NodeBlock::RandPos()
{
    return rand() % 100;
}

NodeItem *NodeBlock::CreatNodeItem00(const QString nodename)
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

    return m_NodeItem;
}

NodeItem *NodeBlock::CreatNodeItemIO(QString nodename, int in, int out)
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

    for (int i = 0; i < in; ++i) {
        m_NodeItem->createPortIn(8 + i * 16, QColor(Qt::cyan));
    }
    for (int j = 0; j < out; ++j) {
        m_NodeItem->createPortOut(8 + j *16, QColor(Qt::cyan));
    }

    return m_NodeItem;
}
