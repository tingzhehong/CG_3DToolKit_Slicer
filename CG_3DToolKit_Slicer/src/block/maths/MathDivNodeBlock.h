#ifndef MATHDIVNODEBLOCK_H
#define MATHDIVNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class MathDivNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit MathDivNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~MathDivNodeBlock() = default;

public:
    void Run() override;

private:
    template<typename T>
    T Div(T a, T b)
    {
        return a / b;
    }

};

#endif // MATHDIVNODEBLOCK_H
