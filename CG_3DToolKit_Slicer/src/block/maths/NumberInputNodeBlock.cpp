#include "NumberInputNodeBlock.h"

NumberInputNodeBlock::NumberInputNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounterMinus = nodeview->m_IDCounterMinus;
    NodeItemNumberInput(tr(u8"数值/输入"));
}

void NumberInputNodeBlock::Run()
{

}
