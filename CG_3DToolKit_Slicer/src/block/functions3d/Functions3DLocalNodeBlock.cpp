#include "Functions3DLocalNodeBlock.h"
#include "CGMetaType.h"

Functions3DLocalNodeBlock::Functions3DLocalNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    NodeItemFactory(tr(u8"3D本地点云"), 0, 1);

    m_NodeItem->m_portList.at(0)->setColor(Qt::red);
}

void Functions3DLocalNodeBlock::Run()
{
    m_NodeItem->PortClass();

    if (m_NodeItem->m_OutPortItem.size() != 1) return;
    if (!m_PointCloud) return;

    m_NodeItem->m_OutPortItem.at(0)->setValue(QVariant::fromValue(m_PointCloud));
    m_IsRuned = true;
}
