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

signals:
    void SignalMessage(const QString msg);
    void SignalComputed();
    
public:
    void Run() override;

private:
    QString RunScript(QScriptEngine &eng, QScriptValue &func, QScriptValueList &args);

};

#endif // LOGICSSCRIPTCPP_H
