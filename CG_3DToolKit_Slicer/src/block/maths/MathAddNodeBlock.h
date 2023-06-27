#ifndef MATHADDNODEBLOCK_H
#define MATHADDNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class MathAddNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit MathAddNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~MathAddNodeBlock() = default;

public:
    void Run() override;

private:
    template<typename T>
    T Add(T a, T b)
    {
        return a + b;
    }

};

#endif // MATHADDNODEBLOCK_H
