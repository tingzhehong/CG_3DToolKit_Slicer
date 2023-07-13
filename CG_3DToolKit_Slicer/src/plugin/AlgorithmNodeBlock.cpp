#include "AlgorithmNodeBlock.h"
#include <QDebug>

AlgorithmNodeBlock::AlgorithmNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;
    m_plugin = nullptr;
}

void AlgorithmNodeBlock::Run()
{
    m_NodeItem->PortClass();

    QVector<QVariant> InputData;
    foreach (PortItem *InPortItem, m_NodeItem->m_InPortItem)
    {
       InputData.push_back(InPortItem->value());
    }
    m_plugin->SetAlgorithmInputData(InputData);
    m_plugin->Compute();

    QVector<QVariant> OutputData;
    OutputData = m_plugin->GetAlgorithmOutputData();

    int m = m_NodeItem->m_OutPortItem.size();
    int n = OutputData.size();
    if (m != n)
    {
        qDebug() << "Algorithm node block data quantity mismatch!" ;
        return;
    }

    for (int i = 0; i < m; ++i)
    {
        QVariant value = OutputData[i];
        m_NodeItem->m_OutPortItem.at(i)->setValue(value);
    }
}

void AlgorithmNodeBlock::SetPlugin(AlgorithmInterface *plugin)
{
    m_plugin = plugin;
}

AlgorithmInterface *AlgorithmNodeBlock::GetPlugin() const
{
    return m_plugin;
}
