#include "Functions2DTerminalNodeBlock.h"
#include "CGMetaType.h"
#include <QLabel>
#include <QDebug>

Functions2DTerminalNodeBlock::Functions2DTerminalNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    NodeItemFactory(tr(u8"2D数据终端"), 1, 0);
}

void Functions2DTerminalNodeBlock::Run()
{
    m_NodeItem->PortClass();

    if (m_NodeItem->m_InPortItem.size() != 1) return;
    QVariant var = m_NodeItem->m_InPortItem.at(0)->value();
    cv::Mat img = var.value<cv::Mat>();
    g_Image.ColorImage = img.clone();
    emit SignalShow2D();
    m_IsRuned = true;
}

NodeItem *Functions2DTerminalNodeBlock::CreatNodeItem10(const QString nodename)
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
