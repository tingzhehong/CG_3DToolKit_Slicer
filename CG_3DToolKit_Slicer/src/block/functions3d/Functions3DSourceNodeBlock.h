#ifndef FUNCTIONS3DSOURCENODEBLOCK_H
#define FUNCTIONS3DSOURCENODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class Functions3DSourceNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit Functions3DSourceNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~Functions3DSourceNodeBlock() = default;

public:
    void Run() override;

protected:
    NodeItem *CreatNodeItem01(const QString nodename) override;

private:

};

#endif // FUNCTIONS3DSOURCENODEBLOCK_H
