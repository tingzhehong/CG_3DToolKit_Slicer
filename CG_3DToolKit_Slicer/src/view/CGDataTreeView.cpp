#include "CGDataTreeView.h"
#include <QLabel>
#include <QTreeWidget>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

CGDataTreeView::CGDataTreeView(CGBaseTreeWidget *parent) : CGBaseTreeWidget(parent)
{
    InitUi();
    InitConnections();
}

CGDataTreeView::~CGDataTreeView()
{
    delete m_DataTree;
    m_DataTree = nullptr;
}

void CGDataTreeView::InitUi()
{
    m_DataTree = new QTreeWidget(this);
    m_DataTree->setHeaderLabel(tr(u8"数据列表"));

    QVBoxLayout *pTreeLayout = new QVBoxLayout();
    pTreeLayout->addWidget(m_DataTree);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addLayout(pTreeLayout);

    setLayout(pMainLayout);
    setVisible(true);
}

void CGDataTreeView::InitConnections()
{

}
