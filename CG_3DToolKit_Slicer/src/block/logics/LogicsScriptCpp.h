#ifndef LOGICSSCRIPTCPP_H
#define LOGICSSCRIPTCPP_H

#include <QObject>
#include <NodeBlock.h>

class LogicsScriptCpp : public NodeBlock
{
    Q_OBJECT

public:
    explicit LogicsScriptCpp(NodeView *nodeview, QWidget *parent = nullptr);
    ~LogicsScriptCpp() = default;

public:
    void Run() override;

private:

};

#endif // LOGICSSCRIPTCPP_H
