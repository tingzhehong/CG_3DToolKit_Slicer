#ifndef FUNCTIONS3DLOCALNODEBLOCK_H
#define FUNCTIONS3DLOCALNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class Functions3DLocalNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit Functions3DLocalNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~Functions3DLocalNodeBlock() = default;

public:
    void Run() override;

private:

};

#endif // FUNCTIONS3DLOCALNODEBLOCK_H
