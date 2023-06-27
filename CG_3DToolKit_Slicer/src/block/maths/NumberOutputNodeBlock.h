#ifndef NUMBEROUTPUTNODEBLOCK_H
#define NUMBEROUTPUTNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class NumberOutputNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit NumberOutputNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~NumberOutputNodeBlock() = default;

public:
    void Run() override;
};

#endif // NUMBEROUTPUTNODEBLOCK_H
