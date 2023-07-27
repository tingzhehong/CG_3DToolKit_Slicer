#include "LogicsCirculate.h"
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QIcon>
#include <QDebug>

LogicsCirculate::LogicsCirculate(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    mainnode = NodeItemFactory(tr(u8"循环"), 1, 0);
    group = m_NodeView->createGroup(QList<NodeItem*>{mainnode});
    group->setTitle(u8"Group 循环");

    Connections();
}

void LogicsCirculate::Connections()
{
    connect(addNode, &QPushButton::clicked, this, &LogicsCirculate::AddNodeBlock);
    connect(delNode, &QPushButton::clicked, this, &LogicsCirculate::DelNodeBlock);
}

void LogicsCirculate::Run()
{
    m_RunBlockList.clear();
    m_NodeItem->PortClass();

    int itimes = m_NodeItem->m_InPortItem[0]->value().toInt();
    int inumbers = group->nodeList().size();

    for (int k = 0; k < inumbers; ++k)
    {
        NodeItem *item = group->nodeList().at(k);
        unsigned int id = item->m_NodeID;

        foreach (NodeBlock* block, m_NodeBlockList)
        {
            if (id == block->m_NodeItem->m_NodeID)
            {
                m_RunBlockList.append(block);
            }
        }
    }

    for (int i = 0; i < itimes; ++i)
    {
        for (int j = 0; j < m_RunBlockList.size(); ++j)
        {
            NodeBlock* block = m_RunBlockList.at(j);

            if (block->IsValid())
                block->Run();
        }
    }

    m_IsRuned = true;
}

void LogicsCirculate::AddNodeBlock()
{
    unsigned int id = lineEdit->text().trimmed().toUInt();
    foreach (NodeItem *node, m_NodeView->m_nodeList)
    {
        if (node->m_NodeID == id && node->m_NodeID != 0) {
            node->m_GroupNode = true;
            group->addNode(node);
        }
    }
    lineEdit->clear();
}

void LogicsCirculate::DelNodeBlock()
{
    unsigned int id = lineEdit->text().trimmed().toUInt();
    foreach (NodeItem *node, group->m_nodeList)
    {
        if (node->m_NodeID == id && node->m_NodeID != 0) {
            node->m_GroupNode = false;
            group->removeNode(node);
         }
    }
    lineEdit->clear();
}

void LogicsCirculate::FillNodeBlock(QVector<NodeBlock*> list)
{
    m_NodeBlockList.clear();

    foreach (NodeBlock* block, list)
    {
       m_NodeBlockList.append(block);
    }
}

NodeItem *LogicsCirculate::CreatNodeItem10(const QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *label = new QLabel(tr(u8"ID:"), widget);
    label->resize(60, 20);
    label->move(2, 15);

    lineEdit = new QLineEdit("", widget);
    lineEdit->resize(36, 20);
    lineEdit->move(32, 15);

    addNode = new QPushButton("+", widget);
    delNode = new QPushButton("-", widget);
    addNode->resize(32, 20);
    delNode->resize(32, 20);
    addNode->move(80, 15);
    delNode->move(115, 15);
    addNode->setToolTip(tr(u8"添加算子"));
    delNode->setToolTip(tr(u8"移除算子"));
    //addNode->setIcon(QIcon(":/res/icon/ccPlus.png"));
    //delNode->setIcon(QIcon(":/res/icon/ccMinus.png"));

    m_NodeItem = m_NodeView->createNode(widget);
    m_NodeItem->setTitle(nodename);
    m_NodeItem->setNodeName(nodename);
    m_NodeItem->setNodeID(m_IDCounter);

    int x = RandPos();
    int y = RandPos();
    m_NodeItem->setPos(x, y);

    m_NodeItem->createPortIn(8, QColor(Qt::cyan));

    return m_NodeItem;
}
