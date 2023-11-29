#ifndef VALUETERMINALNODEBLOCK_H
#define VALUETERMINALNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class ValueTerminalNodeBlock : public NodeBlock
{
    Q_OBJECT
public:
    explicit ValueTerminalNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~ValueTerminalNodeBlock() = default;

signals:
    void SignalShowValue();

public:
    void Run() override;

protected:
    NodeItem *CreatNodeItem10(const QString nodename) override;

private:

};

#endif // VALUETERMINALNODEBLOCK_H
