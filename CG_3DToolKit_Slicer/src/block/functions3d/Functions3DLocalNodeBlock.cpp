#include "Functions3DLocalNodeBlock.h"

Functions3DLocalNodeBlock::Functions3DLocalNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    NodeItemFactory(tr(u8"3D本地点云"), 0, 1);

    m_NodeItem->m_portList.at(0)->setColor(Qt::red);
}

void Functions3DLocalNodeBlock::Run()
{

}
