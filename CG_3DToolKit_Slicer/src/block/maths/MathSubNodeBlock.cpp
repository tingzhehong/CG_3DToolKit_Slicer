﻿#include "MathSubNodeBlock.h"

MathSubNodeBlock::MathSubNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    NodeItemFactory(tr(u8"减"), 2, 1);
}

void MathSubNodeBlock::Run()
{
    m_NodeItem->PortClass();

    if (m_NodeItem->m_InPortItem.size() != 2) return;
    if (m_NodeItem->m_OutPortItem.size() != 1) return;

    float In_0 = m_NodeItem->m_InPortItem[0]->value().toFloat();
    float In_1 = m_NodeItem->m_InPortItem[1]->value().toFloat();
    float Out = Sub(In_0, In_1);

    m_NodeItem->m_OutPortItem[0]->setValue(Out);
    m_IsRuned = true;
}
