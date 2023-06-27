#ifndef MATHMULNODEBLOCK_H
#define MATHMULNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class MathMulNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit MathMulNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~MathMulNodeBlock() = default;

public:
    void Run() override;

private:
    template<typename T>
    T Mul(T a, T b)
    {
        return a * b;
    }

};

#endif // MATHMULNODEBLOCK_H
