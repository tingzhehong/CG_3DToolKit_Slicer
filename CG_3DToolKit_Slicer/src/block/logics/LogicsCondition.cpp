#include "LogicsCondition.h"
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>

LogicsCondition::LogicsCondition(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    NodeItemFactory(tr(u8"条件"), 2, 1);
}

void LogicsCondition::Run()
{
    m_NodeItem->PortClass();

    if (m_NodeItem->m_InPortItem.size() != 2) return;
    if (m_NodeItem->m_OutPortItem.size() != 1) return;

    float In_0 = m_NodeItem->m_InPortItem[0]->value().toFloat();
    float In_1 = m_NodeItem->m_InPortItem[1]->value().toFloat();
    float Out = 0;

    switch (comb->currentIndex())
    {
    case 0:
        Out = Equal(In_0, In_1);
        break;
    case 1:
        Out = Greater(In_0, In_1);
        break;
    case 2:
        Out = Less(In_0, In_1);
        break;
    case 3:
        Out = GreaterEqual(In_0, In_1);
        break;
    case 4:
        Out = LessEqual(In_0, In_1);
        break;
    case 5:
        Out = NotEqual(In_0, In_1);
        break;
    default:
        break;
    }

    m_NodeItem->m_OutPortItem[0]->setValue(Out);
    m_IsRuned = true;
}

NodeItem *LogicsCondition::CreatNodeItem21(const QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *labelId = new QLabel("ID: " + QString::number(m_IDCounter), widget);
    labelId->resize(60, 20);
    labelId->move(2, 15);

    comb = new QComboBox(widget);
    comb->resize(80, 22);
    comb->move(50, 15);
    comb->addItem("    = =  ");
    comb->addItem("    >    ");
    comb->addItem("    <    ");
    comb->addItem("    > =  ");
    comb->addItem("    < =  ");
    comb->addItem("    ! =  ");

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
