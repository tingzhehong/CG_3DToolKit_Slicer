#include "LogicsScriptCpp.h"
#include "CGMetaType.h"
#include <QDebug>

LogicsScriptCpp::LogicsScriptCpp(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    NodeItemFactory(tr(u8"脚本"), 0, 0);
}

void LogicsScriptCpp::Run()
{
    if (m_NodeItem->portList().size() == 0) return;

    //脚本代码
    QVariant var = m_NodeItem->m_Parameters.value(u8"代码");
    QString code = var.toString();
//  qDebug() << code;

    //脚本引擎
    QScriptEngine eng;
    QScriptValue func;

    //设置脚本
    eng.evaluate(code);
    func = eng.globalObject().property("ScriptCpp");

    //设置参数
    QScriptValueList args;
    QScriptValue input;
    QScriptValue output;
    QVariantList inputList, outputList;

    m_NodeItem->PortClass();

    for (int i = 0; i < m_NodeItem->m_InPortItem.size(); ++i)
        inputList << m_NodeItem->m_InPortItem[i]->value();
    for (int j = 0; j < m_NodeItem->m_OutPortItem.size(); ++j)
        outputList << m_NodeItem->m_OutPortItem[j]->value();

    input = eng.newArray(inputList.count());
    output = eng.newArray(outputList.count());

    for (int m = 0; m < inputList.count(); ++m) {
        QScriptValue val = eng.newVariant(inputList[m]);
        QColor clr = m_NodeItem->m_InPortItem[m]->color();
        if (clr == Qt::cyan)
            input.setProperty(m, val.toNumber());
        if (clr == Qt::yellow || clr == Qt::red)
            input.setProperty(m, val.toQMetaObject());
    }

    for (int n = 0; n < outputList.count(); ++n) {
        QScriptValue val = eng.newVariant(outputList[n]);
        QColor clr = m_NodeItem->m_OutPortItem[n]->color();
        if (clr == Qt::cyan)
            output.setProperty(n, val.toNumber());
        if (clr == Qt::yellow || clr == Qt::red)
            output.setProperty(n, val.toQMetaObject());
    }

    args.append(input);
    args.append(output);

    //执行脚本
    QString ret = RunScript(eng, func, args);
    qDebug() << ret;

    //脚本输出
    for (int k = 0; k < outputList.count(); ++k)
    {
        QVariant var = args[1].property(k).toVariant();
        m_NodeItem->m_OutPortItem[k]->setValue(var);
    }
}

QString LogicsScriptCpp::RunScript(QScriptEngine &eng, QScriptValue &func, QScriptValueList &args)
{
    //返回信息
    QString str;
    //执行脚本
    QScriptValue result = func.call(QScriptValue(), args);
    //异常输出
    if (eng.hasUncaughtException())
    {
        str += "异常：" + eng.uncaughtException().toString() + "\n";
        str += "堆栈：" + eng.uncaughtExceptionBacktrace().join("/n");
        str += QString("异常位置：第%1行\n").arg(eng.uncaughtExceptionLineNumber());
    }
    else
    {
        str = result.toString();
    }

    return str;
}
