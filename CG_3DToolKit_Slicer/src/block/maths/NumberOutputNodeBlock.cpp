#include "NumberOutputNodeBlock.h"

NumberOutputNodeBlock::NumberOutputNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    NodeItemNumberOutput(tr(u8"数值/输出"));
}

void NumberOutputNodeBlock::Run()
{

}
