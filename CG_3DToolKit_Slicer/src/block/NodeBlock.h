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

    bool IsValid();
    bool IsRuned();
    int  RandPos();

    virtual void Run() = 0;

public:
    NodeView *m_NodeView;
    NodeItem *m_NodeItem;

    unsigned int m_IDCounter;
    		 int m_IDCounterMinus;
    bool m_IsRuned;

private:
    virtual NodeItem *CreatNodeItem00(const QString nodename);
    virtual NodeItem *CreatNodeItem01(const QString nodename);
    virtual NodeItem *CreatNodeItem10(const QString nodename);
    virtual NodeItem *CreatNodeItem11(const QString nodename);
    virtual NodeItem *CreatNodeItem21(const QString nodename);
    virtual NodeItem *CreatNodeItem31(const QString nodename);
    virtual NodeItem *CreatNodeItem41(const QString nodename);
    virtual NodeItem *CreatNodeItem12(const QString nodename);
    virtual NodeItem *CreatNodeItem22(const QString nodename);
    virtual NodeItem *CreatNodeItem33(const QString nodename);

};

#endif // NODEBLOCK_H
