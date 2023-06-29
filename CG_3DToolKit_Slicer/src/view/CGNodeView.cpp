#include "CGNodeView.h"
#include <QLabel>
#include <QLineEdit>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>
#include <QDebug>
#include <memory>
#include "NumberInputNodeBlock.h"
#include "NumberOutputNodeBlock.h"
#include "MathAddNodeBlock.h"
#include "MathDivNodeBlock.h"
#include "MathMulNodeBlock.h"
#include "MathSubNodeBlock.h"
#include "LogicsCondition.h"
#include "LogicsCirculate.h"
#include "Functions2DSourceNodeBlock.h"
#include "Functions3DSourceNodeBlock.h"


CGNodeView::CGNodeView(QWidget *parent) : CGBaseWidget(parent)
{
    InitUi();
    InitConnections();
    setWindowTitle(tr(u8"流程节点"));
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
}

CGNodeView::~CGNodeView()
{

}

void CGNodeView::InitUi()
{
    m_NodeView = new NodeView(this);
    m_NodeView->setRopeFlexion(50);

    //Test();
    //Verify();

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_NodeView);

    setLayout(pMainLayout);
}

void CGNodeView::InitConnections()
{
    connect(m_NodeView, &NodeView::signalRemoveNode, this, &CGNodeView::OnRemoveNodeBlock);

}

void CGNodeView::CreateMathsNodeItem(const QString toolname)
{
         if (toolname == u8"数值/输入") {
            NumberInputNodeBlock *input = new NumberInputNodeBlock(m_NodeView);
            m_NodeBlockList.append(dynamic_cast<NodeBlock*>(input));
         }
    else if (toolname == u8"数值/输出") {
            NumberOutputNodeBlock *output = new NumberOutputNodeBlock(m_NodeView);
            m_NodeBlockList.append(dynamic_cast<NodeBlock*>(output));
         }
    else if (toolname == u8"加") {
            MathAddNodeBlock *add = new MathAddNodeBlock(m_NodeView);
            m_NodeBlockList.append(dynamic_cast<NodeBlock*>(add));
            m_NodeView->m_IDCounter++;
         }
    else if (toolname == u8"减") {
            MathSubNodeBlock *sub = new MathSubNodeBlock(m_NodeView);
            m_NodeBlockList.append(dynamic_cast<NodeBlock*>(sub));
            m_NodeView->m_IDCounter++;
         }
    else if (toolname == u8"乘") {
            MathMulNodeBlock *mul = new MathMulNodeBlock(m_NodeView);
            m_NodeBlockList.append(dynamic_cast<NodeBlock*>(mul));
            m_NodeView->m_IDCounter++;
         }
    else if (toolname == u8"除") {
            MathDivNodeBlock *div = new MathDivNodeBlock(m_NodeView);
            m_NodeBlockList.append(dynamic_cast<NodeBlock*>(div));
            m_NodeView->m_IDCounter++;
         }
    else {
            ;   
         }

}

void CGNodeView::CreateLogicsNodeItem(const QString toolname)
{
    if (toolname == u8"条件") {
        LogicsCondition *condition = new LogicsCondition(m_NodeView);
        m_NodeBlockList.append(dynamic_cast<NodeBlock*>(condition));
        m_NodeView->m_IDCounter++;
    }
    if (toolname == u8"循环") {
        LogicsCirculate *circulate = new LogicsCirculate(m_NodeView);
        m_NodeBlockList.append(dynamic_cast<NodeBlock*>(circulate));
        m_NodeView->m_IDCounter++;
    }

}

void CGNodeView::Create2DFuctionNodeItem(const QString toolname, int index)
{
    if (toolname == u8"2D数据源") {
        Functions2DSourceNodeBlock *src2d = new Functions2DSourceNodeBlock(m_NodeView);
        m_NodeBlockList.append(dynamic_cast<NodeBlock*>(src2d));
        m_NodeView->m_IDCounter++;
    }
    Q_UNUSED(index);
}

void CGNodeView::Create3DFuctionNodeItem(const QString toolname, int index)
{
    if (toolname == u8"3D数据源") {
        Functions3DSourceNodeBlock *src3d = new Functions3DSourceNodeBlock(m_NodeView);
        m_NodeBlockList.append(dynamic_cast<NodeBlock*>(src3d));
        m_NodeView->m_IDCounter++;
    }
    Q_UNUSED(index);
}

void CGNodeView::Run()
{
    m_RunBlockList.clear();

    foreach (NodeBlock *block, m_NodeBlockList)
    {
        block->m_IsRuned = false;
        if (block->m_NodeItem->m_NodeID != 0)
            m_RunBlockList.append(block);
    }

    int i = 0;
    int j = 0;
    int k = 0;
    int num = m_RunBlockList.size();
    qDebug() << "Run Node Block Size: " << m_RunBlockList.size();

    while (i < RUNMAXLOOP)
    {
        j = 0;

        while (j < num)
        {
            NodeBlock *block = m_RunBlockList.at(j);

            if (block->IsValid() && !block->IsRuned())
            {
                block->Run();
                //qDebug() << block->m_NodeItem->m_OutPortItem.at(0)->value();
                ++k;
            }
            ++j;

            if (j == num)
                break;
        }
        ++i;

        if (k == num || i == RUNMAXLOOP)
            break;

    }
    qDebug() <<"Run Times Count: " << i;

}

void CGNodeView::RunBlockUpdate()
{
    m_RunBlockList.clear();

    foreach (NodeBlock *block, m_NodeBlockList)
    {
        block->m_IsRuned = false;
        if (block->m_NodeItem->m_NodeID != 0)
            m_RunBlockList.append(block);
    }
}

void CGNodeView::Test()
{
    PortItem *portIn = Q_NULLPTR;

    QWidget *widget = new QWidget;
    widget->resize(150, 30);

    QLabel *label = new QLabel("Node:",widget);
    label->resize(60, 20);
    label->move(2, 5);

    QLineEdit *lineEdit = new QLineEdit("123456", widget);
    lineEdit->resize(105, 20);
    lineEdit->move(40, 5);

    NodeItem *node = m_NodeView->createNode(widget);
    node->setTitle("Node Item");
    PortItem *portOut = node->createPortOut(-8, QColor(Qt::cyan));
    node->setPos(-250, 90);

    for (int i=0; i<5; ++i)
    {
        QWidget *widget = new QWidget;
        widget->resize(150, 30);

        QLabel *label = new QLabel("Node:",widget);
        label->resize(60, 20);
        label->move(2, 5);

        QLineEdit *lineEdit = new QLineEdit("123456",widget);
        lineEdit->resize(105, 20);
        lineEdit->move(40, 5);

        NodeItem *node = m_NodeView->createNode(widget);
        node->setTitle("Node Item " + QString::number(i + 1));
        node->setPos(i * 15, i * 60);

        portIn = node->createPortIn(-8, QColor(Qt::cyan));
        m_NodeView->createConnection(portOut, portIn);
    }

    GroupItem *group = m_NodeView->createGroup(m_NodeView->nodeList());
    group->setTitle("Group item");
    group->removeNode(node);
}

void CGNodeView::Verify()
{
    m_NodeView->NodeItemFactory("1#", 1, 1);
    m_NodeView->NodeItemFactory("2#", 2, 1);
    m_NodeView->NodeItemFactory("3#", 3, 1);
}

void CGNodeView::OnRemoveNodeBlock(unsigned int nodeId)
{
    foreach (NodeBlock* block, m_NodeBlockList)
    {
        if (block->m_NodeItem->m_NodeID == nodeId)
        {
            m_NodeBlockList.removeOne(block);
            delete block;
            block = nullptr;
        }
    }
}
