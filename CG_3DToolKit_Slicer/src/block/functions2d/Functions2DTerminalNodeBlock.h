#ifndef FUNCTIONS2DTERMINALNODEBLOCK_H
#define FUNCTIONS2DTERMINALNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class Functions2DTerminalNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit Functions2DTerminalNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~Functions2DTerminalNodeBlock() = default;

public:
    void Run() override;

protected:
    NodeItem *CreatNodeItem10(const QString nodename) override;

private:

};

#endif // FUNCTIONS2DTERMINALNODEBLOCK_H
