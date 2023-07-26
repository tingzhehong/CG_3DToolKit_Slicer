#include "NodeBlockManager.h"
#include <QDebug>

NodeBlockManager::NodeBlockManager(QObject *parent) : QObject(parent)
{

}

void NodeBlockManager::run()
{
    m_RunBlockList.clear();

    foreach (NodeBlock *block, m_NodeBlockList)
    {
        block->m_IsRuned = false;
        if (block->m_NodeItem->m_NodeID > 0)
            m_RunBlockList.append(block);
    }

    int i = 0;
    int j = 0;
    int k = 0;
    int num = m_RunBlockList.size();
    qDebug() << "Run Node Block Size: " << m_RunBlockList.size();

    while (i < num)
    {
        j = 0;

        while (j < num)
        {
            NodeBlock *block = m_RunBlockList.at(j);

            if (block->IsValid() && !block->IsRuned())
            {
                block->Run();
                //qDebug() << block->m_NodeItem->m_OutPortItem.at(0)->value();
                ++k;
            }
            ++j;

            if (j == num)
                break;
        }
        ++i;

        if (k == num || i == num)
            break;

    }
    qDebug() <<"Run Times Count: " << i;
}
