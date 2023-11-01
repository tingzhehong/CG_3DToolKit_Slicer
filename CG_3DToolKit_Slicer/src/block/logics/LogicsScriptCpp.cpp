#include "LogicsScriptCpp.h"
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

    QVariant var = m_NodeItem->m_Parameters.value(u8"代码");
    QString code = var.toString();
    qDebug() << code;

}
