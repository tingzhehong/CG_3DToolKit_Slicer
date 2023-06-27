#include "NumberInputNodeBlock.h"

NumberInputNodeBlock::NumberInputNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    NodeItemNumberInput(tr(u8"数值/输入"));
}

void NumberInputNodeBlock::Run()
{

}
