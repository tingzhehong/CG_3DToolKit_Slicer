#include "AlgorithmNodeBlock.h"

AlgorithmNodeBlock::AlgorithmNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;
    m_plugin = nullptr;
}

void AlgorithmNodeBlock::Run()
{
    m_plugin->Compute();
}

void AlgorithmNodeBlock::SetPlugin(AlgorithmInterface *plugin)
{
    m_plugin = plugin;
}

AlgorithmInterface *AlgorithmNodeBlock::GetPlugin() const
{
    return m_plugin;
}
