#ifndef NODEBLOCK_H
#define NODEBLOCK_H

#include <QWidget>
#include <NodeItem.h>
#include <NodeView.h>


class NodeBlock : public QWidget
{
    Q_OBJECT

public:
    explicit NodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~NodeBlock() = default;

public:
    NodeItem *NodeItemFactory(QString nodename, int in, int out);
    NodeItem *NodeItemNumberInput(QString nodename);
    NodeItem *NodeItemNumberOutput(QString nodename);

    bool Valid();

    virtual void Run() = 0;

public:
    NodeView *m_NodeView;
    NodeItem *m_NodeItem;

    unsigned int m_IDCounter;

private:
    NodeItem *CreatNodeItem01(const QString nodename);
    NodeItem *CreatNodeItem10(const QString nodename);
    NodeItem *CreatNodeItem11(const QString nodename);
    NodeItem *CreatNodeItem21(const QString nodename);
    NodeItem *CreatNodeItem31(const QString nodename);
    NodeItem *CreatNodeItem12(const QString nodename);

    int RandPos();

};

#endif // NODEBLOCK_H
