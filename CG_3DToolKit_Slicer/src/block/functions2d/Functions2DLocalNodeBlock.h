#ifndef FUNCTIONS2DLOCALNODEBLOCK_H
#define FUNCTIONS2DLOCALNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class Functions2DLocalNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit Functions2DLocalNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~Functions2DLocalNodeBlock() = default;

public:
    void Run() override;

private:

};

#endif // FUNCTIONS2DLOCALNODEBLOCK_H
