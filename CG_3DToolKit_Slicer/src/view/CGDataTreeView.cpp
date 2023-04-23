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

    m_ToolBoxTopItem = new QTreeWidgetItem(QStringList{tr(u8"工具箱")});

    m_2DToolBox = new QTreeWidgetItem(QStringList{tr(u8"2D工具箱")});
    for (int i = 0; i < m_2DToolNames.size(); ++i) {
        QTreeWidgetItem *p2DToolItem = new QTreeWidgetItem(QStringList() << m_2DToolNames.at(i));
        m_2DToolBox->addChild(p2DToolItem);
    }

    m_3DToolBox = new QTreeWidgetItem(QStringList{tr(u8"3D工具箱")});
    for (int i = 0; i < m_3DToolNames.size(); ++i) {
        QTreeWidgetItem *p3DToolItem = new QTreeWidgetItem(QStringList() << m_3DToolNames.at(i));
        m_3DToolBox->addChild(p3DToolItem);
    }

    m_ProfileToolBox = new QTreeWidgetItem(QStringList{tr(u8"轮廓工具箱")});
    for (int i = 0; i < m_ProfileToolNames.size(); ++i) {
        QTreeWidgetItem *pProfileToolItem = new QTreeWidgetItem(QStringList() << m_ProfileToolNames.at(i));
        m_ProfileToolBox->addChild(pProfileToolItem);
    }

    m_ToolBoxTopItem->addChild(m_2DToolBox);
    m_ToolBoxTopItem->addChild(m_3DToolBox);
    m_ToolBoxTopItem->addChild(m_ProfileToolBox);


    m_NodeFlowTopItem = new QTreeWidgetItem(QStringList{tr(u8"流程算子")});

    m_Maths = new QTreeWidgetItem(QStringList{tr(u8"数学计算")});
    for (int j = 0; j < m_MathsNames.size(); ++j) {
        QTreeWidgetItem *pMathsItem = new QTreeWidgetItem(QStringList() << m_MathsNames.at(j));
        m_Maths->addChild(pMathsItem);
    }

    m_Logics = new QTreeWidgetItem(QStringList{tr(u8"逻辑条件")});
    for (int j = 0; j < m_LogicsNames.size(); ++j) {
        QTreeWidgetItem *pLogicsItem = new QTreeWidgetItem(QStringList() << m_LogicsNames.at(j));
        m_Logics->addChild(pLogicsItem);
    }

    m_NodeFlowTopItem->addChild(m_Maths);
    m_NodeFlowTopItem->addChild(m_Logics);


    m_FuctionTopItem = new QTreeWidgetItem(QStringList{tr(u8"功能算子")});
    m_2DFuction = new QTreeWidgetItem(QStringList{tr(u8"2D功能算子")});
    m_3DFuction = new QTreeWidgetItem(QStringList{tr(u8"3D功能算子")});

    m_FuctionTopItem->addChild(m_2DFuction);
    m_FuctionTopItem->addChild(m_3DFuction);


    m_DataTree->addTopLevelItem(m_ToolBoxTopItem);
    m_DataTree->addTopLevelItem(m_NodeFlowTopItem);
    m_DataTree->addTopLevelItem(m_FuctionTopItem);
    m_DataTree->expandItem(m_ToolBoxTopItem);
    m_DataTree->expandItem(m_NodeFlowTopItem);
    m_DataTree->expandItem(m_FuctionTopItem);

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
