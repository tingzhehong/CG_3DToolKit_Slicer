#include "NumberOutputNodeBlock.h"

NumberOutputNodeBlock::NumberOutputNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounterMinus = nodeview->m_IDCounterMinus;
    NodeItemNumberOutput(tr(u8"数值/输出"));
}

void NumberOutputNodeBlock::Run()
{

}
