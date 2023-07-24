#include "Functions2DLocalNodeBlock.h"

Functions2DLocalNodeBlock::Functions2DLocalNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    NodeItemFactory(tr(u8"2D本地图像"), 0, 1);

    m_NodeItem->m_portList.at(0)->setColor(Qt::yellow);
}

void Functions2DLocalNodeBlock::Run()
{

}
