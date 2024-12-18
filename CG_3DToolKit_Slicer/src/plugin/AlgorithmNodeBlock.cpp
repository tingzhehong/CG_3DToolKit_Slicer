﻿#include "AlgorithmNodeBlock.h"
#include "CGGlobalVariable.h"
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

    QVector<CG_ARGUMENT> args = m_plugin->GetAlgorithmArguments();
    for (CG_ARGUMENT &parameter : args)
    {
        QString key = parameter.ARG;
        float value = m_NodeItem->m_Parameters.value(key).toFloat();
        parameter.VALUE = value;

        if (key == "XPitch") *g_pXPITCH = value;
        if (key == "YPitch") *g_pYPITCH = value;
    }
    if (args.count() > 0)
        m_plugin->SetAlgorithmArguments(args);

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

    m_IsRuned = true;
}

void AlgorithmNodeBlock::SetPlugin(AlgorithmInterface *plugin)
{
    m_plugin = plugin;
}

AlgorithmInterface *AlgorithmNodeBlock::GetPlugin() const
{
    return m_plugin;
}
