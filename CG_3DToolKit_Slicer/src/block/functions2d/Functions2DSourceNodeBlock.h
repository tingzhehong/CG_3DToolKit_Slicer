#ifndef FUNCTIONS2DSOURCENODEBLOCK_H
#define FUNCTIONS2DSOURCENODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class Functions2DSourceNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit Functions2DSourceNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~Functions2DSourceNodeBlock() = default;

public:
    void Run() override;

protected:
    NodeItem *CreatNodeItem01(const QString nodename) override;

private:

};

#endif // FUNCTIONS2DSOURCENODEBLOCK_H
