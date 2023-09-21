#include "CGDataTreeView.h"
#include <QLabel>
#include <QTreeWidget>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QMenu>
#include <QAction>

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

void CGDataTreeView::OnAlgorithmPluginAdd(QPair<QStringList, QStringList> names)
{
    for (int i = 0; i < names.first.size(); ++i) {
        QTreeWidgetItem *p2DFuctionItem = new QTreeWidgetItem(QStringList() << names.first.at(i));
        m_2DFuction->addChild(p2DFuctionItem);
        m_2DFuctionNames.append(names.first.at(i));
    }
    //qDebug() << m_2DFuctionNames;

    for (int j = 0; j < names.second.size(); ++j) {
        QTreeWidgetItem *p3DFuctionItem = new QTreeWidgetItem(QStringList() << names.second.at(j));
        m_3DFuction->addChild(p3DFuctionItem);
        m_3DFuctionNames.append(names.second.at(j));
    }
    //qDebug() << m_3DFuctionNames;
}

void CGDataTreeView::InitUi()
{
    m_DataTree = new QTreeWidget(this);
    m_DataTree->setHeaderLabel(tr(u8"数据列表"));

///////
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

///////
    m_NodeFlowTopItem = new QTreeWidgetItem(QStringList{tr(u8"流程算子")});

    m_Maths = new QTreeWidgetItem(QStringList{tr(u8"数学计算")});
    for (int j = 0; j < m_MathsNames.size(); ++j) {
        QTreeWidgetItem *pMathsItem = new QTreeWidgetItem(QStringList() << m_MathsNames.at(j));
        m_Maths->addChild(pMathsItem);
    }

    m_Logics = new QTreeWidgetItem(QStringList{tr(u8"逻辑运算")});
    for (int j = 0; j < m_LogicsNames.size(); ++j) {
        QTreeWidgetItem *pLogicsItem = new QTreeWidgetItem(QStringList() << m_LogicsNames.at(j));
        m_Logics->addChild(pLogicsItem);
    }

    m_NodeFlowTopItem->addChild(m_Maths);
    m_NodeFlowTopItem->addChild(m_Logics);

///////
    m_FuctionTopItem = new QTreeWidgetItem(QStringList{tr(u8"功能算子")});
    m_2DFuction = new QTreeWidgetItem(QStringList{tr(u8"2D功能算子")});
    for (int k = 0; k < m_2DFuctionNames.size(); ++k) {
        QTreeWidgetItem *p2DFuctionItem = new QTreeWidgetItem(QStringList() << m_2DFuctionNames.at(k));
        m_2DFuction->addChild(p2DFuctionItem);
    }

    m_3DFuction = new QTreeWidgetItem(QStringList{tr(u8"3D功能算子")});
    for (int k = 0; k < m_3DFuctionNames.size(); ++k) {
        QTreeWidgetItem *p3DFuctionItem = new QTreeWidgetItem(QStringList() << m_3DFuctionNames.at(k));
        m_3DFuction->addChild(p3DFuctionItem);
    }

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

    m_action2DToolClear = new QAction(tr(u8"清除2D工具"));
    m_action3DToolClear = new QAction(tr(u8"清除3D工具"));
    m_actionProfileToolClear = new QAction(tr(u8"清除轮廓工具"));
    m_DataTree->addAction(m_action2DToolClear);
    m_DataTree->addAction(m_action3DToolClear);
    m_DataTree->addAction(m_actionProfileToolClear);
    m_DataTree->setContextMenuPolicy(Qt::ActionsContextMenu);
}

void CGDataTreeView::InitConnections()
{
    connect(m_action2DToolClear, &QAction::triggered, [this](){emit Signal2DToolClear();});
    connect(m_action3DToolClear, &QAction::triggered, [this](){emit Signal3DToolClear();});
    connect(m_actionProfileToolClear, &QAction::triggered, [this](){emit SignalProfileToolClear();});
}

