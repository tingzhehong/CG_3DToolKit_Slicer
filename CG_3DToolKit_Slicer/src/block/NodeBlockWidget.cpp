#include "NodeBlockWidget.h"

NodeBlockWidget *NodeBlockWidget::m_NodeBlockWidget = nullptr;

NodeBlockWidget::NodeBlockWidget(QWidget *parent) : QWidget(parent)
{

}

NodeBlockWidget *NodeBlockWidget::getInstance()
{
    if (!m_NodeBlockWidget)
    {
        m_NodeBlockWidget = new NodeBlockWidget();
    }
    return m_NodeBlockWidget;
}

void NodeBlockWidget::InitUi()
{

}

void NodeBlockWidget::InitConnections()
{

}
