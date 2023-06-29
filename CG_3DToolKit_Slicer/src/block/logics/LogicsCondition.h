#ifndef LOGICSCONDITION_H
#define LOGICSCONDITION_H

#include <QObject>
#include <NodeBlock.h>

class QComboBox;
class LogicsCondition : public NodeBlock
{
    Q_OBJECT

public:
    explicit LogicsCondition(NodeView *nodeview, QWidget *parent = nullptr);
    ~LogicsCondition() = default;

public:
    void Run() override;

protected:
    NodeItem *CreatNodeItem21(const QString nodename) override;

private:
    QComboBox *comb;

private:
    // == 0
    template<typename T>
    float Equal(T a, T b)
    {
        if (a == b)
            return 1;
        else
            return 0;
    }

    // >  1
    template<typename T>
    float Greater(T a, T b)
    {
        if (a > b)
            return 1;
        else
            return 0;
    }

    // <  2
    template<typename T>
    float Less(T a, T b)
    {
        if (a < b)
            return 1;
        else
            return 0;
    }

    // >=  3
    template<typename T>
    float GreaterEqual(T a, T b)
    {
        if (a >= b)
            return 1;
        else
            return 0;
    }

    // <=  4
    template<typename T>
    float LessEqual(T a, T b)
    {
        if (a <= b)
            return 1;
        else
            return 0;
    }

    // !=  5
    template<typename T>
    float NotEqual(T a, T b)
    {
        if (a != b)
            return 1;
        else
            return 0;
    }
};

#endif // LOGICSCONDITION_H
