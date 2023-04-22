#include "CGProjectTreeView.h"
#include <QLabel>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

CGProjectTreeView::CGProjectTreeView(CGBaseTreeWidget *parent) : CGBaseTreeWidget(parent)
{
    InitUi();
    InitConnections();
}

CGProjectTreeView::~CGProjectTreeView()
{
    delete m_ProjectTree;
    m_ProjectTree = nullptr;
}

void CGProjectTreeView::InitUi()
{
    m_ProjectTree = new QTreeWidget(this);
    m_ProjectTree->setHeaderLabel(tr(u8"项目列表"));

    QTreeWidgetItem *ImageTopItem = new QTreeWidgetItem(QStringList{tr(u8"显示项目")});
    m_Window2D = new QTreeWidgetItem(QStringList{tr(u8"2D  图像")});
    m_Window3D = new QTreeWidgetItem(QStringList{tr(u8"3D  图像")});

    QTreeWidgetItem *EditionTopItem = new QTreeWidgetItem(QStringList{tr(u8"流程项目")});
    m_WindowNodeEdit = new QTreeWidgetItem(QStringList{tr(u8"流程节点")});

    QTreeWidgetItem *ProfileTopTtem = new QTreeWidgetItem(QStringList{tr(u8"分析项目")});
    m_WindowProfile = new QTreeWidgetItem(QStringList{tr(u8"轮廓分析")});

    ImageTopItem->addChild(m_Window2D);
    ImageTopItem->addChild(m_Window3D);
    EditionTopItem->addChild(m_WindowNodeEdit);
    ProfileTopTtem->addChild(m_WindowProfile);
    m_ProjectTree->addTopLevelItem(ImageTopItem);
    m_ProjectTree->addTopLevelItem(EditionTopItem);
    m_ProjectTree->addTopLevelItem(ProfileTopTtem);
    m_ProjectTree->expandAll();

    QVBoxLayout *pTreeLayout = new QVBoxLayout();
    pTreeLayout->addWidget(m_ProjectTree);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addLayout(pTreeLayout);

    setLayout(pMainLayout);
    setVisible(true);
}

void CGProjectTreeView::InitConnections()
{

}
