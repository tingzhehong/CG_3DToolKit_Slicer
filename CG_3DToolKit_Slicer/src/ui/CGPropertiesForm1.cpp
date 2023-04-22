#include "CGPropertiesForm1.h"
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QDebug>

CGPropertiesForm1::CGPropertiesForm1(QWidget *parent) : QWidget(parent)
{
    InitUi();
}

void CGPropertiesForm1::InitUi()
{
    m_NullTree = new QTreeWidget(this);
    m_NullTree->setHeaderLabel(tr(u8"属性列表"));

    QVBoxLayout *pTreeLayout = new QVBoxLayout();
    pTreeLayout->addWidget(m_NullTree);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addLayout(pTreeLayout);

    setLayout(pMainLayout);
    setVisible(true);
}
