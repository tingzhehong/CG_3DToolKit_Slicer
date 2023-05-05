#include "NodeBlock.h"
#include <QDebug>

NodeBlock::NodeBlock(NodeView *nodeview, QWidget *parent) : QWidget(parent)
{
    m_NodeView = nodeview;
}

