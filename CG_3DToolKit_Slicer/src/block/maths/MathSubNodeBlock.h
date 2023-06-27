#ifndef MATHSUBNODEBLOCK_H
#define MATHSUBNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>

class MathSubNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit MathSubNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~MathSubNodeBlock() = default;

public:
    void Run() override;

private:
    template<typename T>
    T Sub(T a, T b)
    {
        return a - b;
    }

};

#endif // MATHSUBNODEBLOCK_H
