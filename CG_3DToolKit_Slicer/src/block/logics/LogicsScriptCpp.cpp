#include "LogicsScriptCpp.h"

LogicsScriptCpp::LogicsScriptCpp(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    NodeItemFactory(tr(u8"脚本"), 0, 0);
}

void LogicsScriptCpp::Run()
{

}
