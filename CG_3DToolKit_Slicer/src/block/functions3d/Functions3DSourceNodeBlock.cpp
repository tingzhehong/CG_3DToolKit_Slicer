#include "Functions3DSourceNodeBlock.h"
#include "CGMetaType.h"
#include <QLabel>

Functions3DSourceNodeBlock::Functions3DSourceNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    NodeItemFactory(tr(u8"3D数据源"), 0, 1);
}

void Functions3DSourceNodeBlock::Run()
{
    m_NodeItem->PortClass();

    if (m_NodeItem->m_OutPortItem.size() != 1) return;
    if (!g_PointCloud) return;

    m_NodeItem->m_OutPortItem.at(0)->setValue(QVariant::fromValue(g_PointCloud));
    m_IsRuned = true;
}

NodeItem *Functions3DSourceNodeBlock::CreatNodeItem01(const QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *labelId = new QLabel("ID: " + QString::number(m_IDCounter), widget);
    labelId->resize(60, 20);
    labelId->move(2, 15);

    QLabel *label = new QLabel(tr(u8"算子: ") + nodename, widget);
    label->resize(90, 20);
    label->move(56, 15);

    m_NodeItem = m_NodeView->createNode(widget);
    m_NodeItem->setTitle(nodename);
    m_NodeItem->setNodeName(nodename);
    m_NodeItem->setNodeID(m_IDCounter);

    int x = RandPos();
    int y = RandPos();
    m_NodeItem->setPos(x, y);

    m_NodeItem->createPortOut(8, QColor(Qt::red));

    return m_NodeItem;
}
