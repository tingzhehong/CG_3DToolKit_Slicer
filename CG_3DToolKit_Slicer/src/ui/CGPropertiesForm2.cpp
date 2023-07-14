#include "CGPropertiesForm2.h"
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QDebug>

CGPropertiesForm2::CGPropertiesForm2(QWidget *parent) : QWidget(parent)
{
    InitUi();
}

void CGPropertiesForm2::InitUi()
{
    m_PropertiesTree = new QTreeWidget(this);
    m_PropertiesTree->setHeaderLabel(tr(u8"数据属性"));

    QVBoxLayout *pTreeLayout = new QVBoxLayout();
    pTreeLayout->addWidget(m_PropertiesTree);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addLayout(pTreeLayout);

    setLayout(pMainLayout);
}
