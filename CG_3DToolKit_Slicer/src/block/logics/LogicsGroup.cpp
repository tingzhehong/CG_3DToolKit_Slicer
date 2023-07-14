#include "LogicsGroup.h"
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QIcon>

LogicsGroup::LogicsGroup(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    mainnode = NodeItemFactory(tr(u8"组(主控节点)"), 0, 0);
    group = m_NodeView->createGroup(QList<NodeItem*>{mainnode});
    group->setTitle(u8"Group 组");

    Connections();
}

void LogicsGroup::Connections()
{
    connect(addNode, &QPushButton::clicked, this, &LogicsGroup::AddNodeBlock);
    connect(delNode, &QPushButton::clicked, this, &LogicsGroup::DelNodeBlock);
}

void LogicsGroup::Run()
{

}

void LogicsGroup::AddNodeBlock()
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

void LogicsGroup::DelNodeBlock()
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

NodeItem *LogicsGroup::CreatNodeItem00(const QString nodename)
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
    m_NodeItem->setNodeID(0);

    int x = RandPos();
    int y = RandPos();
    m_NodeItem->setPos(x, y);

    return m_NodeItem;
}
