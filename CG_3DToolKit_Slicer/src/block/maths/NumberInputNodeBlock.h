#ifndef NUMBERINPUTNODEBLOCK_H
#define NUMBERINPUTNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class NumberInputNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit NumberInputNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~NumberInputNodeBlock() = default;

public:
    void Run() override;
};

#endif // NUMBERINPUTNODEBLOCK_H
