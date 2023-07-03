#ifndef FUNCTIONS3DTERMINALNODEBLOCK_H
#define FUNCTIONS3DTERMINALNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class Functions3DTerminalNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit Functions3DTerminalNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~Functions3DTerminalNodeBlock() = default;

signals:
    void SignalShow3D();

public:
    void Run() override;

protected:
    NodeItem *CreatNodeItem10(const QString nodename) override;

private:

};

#endif // FUNCTIONS3DTERMINALNODEBLOCK_H
