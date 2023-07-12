#include "NodeBlockFactory.h"
#include <AlgorithmNodeBlock.h>

NodeBlockFactory::NodeBlockFactory(QObject *parent) : QObject(parent)
{

}

NodeBlock* NodeBlockFactory::CreatNodeBlock(AlgorithmInterface *plugin, CG_NODEBLOCK *pluginobj, NodeView *nodeview)
{
    int iInput = pluginobj->Input.size();
    int iOutput = pluginobj->Output.size();

    AlgorithmNodeBlock *block = new AlgorithmNodeBlock(nodeview);
    block->SetPlugin(plugin);
    block->NodeItemFactory(pluginobj->Name, iInput, iOutput);
    block->m_NodeItem->PortClass();

    for (int i = 0; i < iInput; ++i)
        block->m_NodeItem->m_InPortItem[i]->setColor(pluginobj->Input[i].CLR);

    for (int j = 0; j < iOutput; ++j)
        block->m_NodeItem->m_OutPortItem[j]->setColor(pluginobj->Output[j].CLR);

    return dynamic_cast<NodeBlock*>(block);
}
