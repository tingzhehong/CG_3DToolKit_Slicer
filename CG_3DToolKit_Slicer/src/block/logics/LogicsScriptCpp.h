#ifndef LOGICSSCRIPTCPP_H
#define LOGICSSCRIPTCPP_H

#include <QObject>
#include <NodeBlock.h>
#include <QScriptEngine>
#include <QScriptValue>
#include <QScriptValueList>

class LogicsScriptCpp : public NodeBlock
{
    Q_OBJECT

public:
    explicit LogicsScriptCpp(NodeView *nodeview, QWidget *parent = nullptr);
    ~LogicsScriptCpp() = default;

public:
    void Run() override;

private:
    QString RunScript(QScriptEngine &eng, QScriptValue &func, QScriptValueList &args);

public:
    Q_INVOKABLE double ScriptAdd(double a, double b);
    Q_INVOKABLE double ScriptSub(double a, double b);
    Q_INVOKABLE double ScriptMul(double a, double b);
    Q_INVOKABLE double ScriptDiv(double a, double b);
};

#endif // LOGICSSCRIPTCPP_H
