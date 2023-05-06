#include "CGNodeView.h"
#include <QLabel>
#include <QLineEdit>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>
#include <QDebug>

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
//  Test();
//  Verify();

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_NodeView);

    setLayout(pMainLayout);
}

void CGNodeView::InitConnections()
{

}

void CGNodeView::CreateMathsNodeItem(const QString toolname)
{
         if (toolname == u8"数值/输入")
        m_NodeView->NodeItemNumberInput(toolname);
    else if (toolname == u8"数值/输出")
        m_NodeView->NodeItemNumberOutput(toolname);
    else
        m_NodeView->NodeItemFactory(toolname, 2, 1);
}

void CGNodeView::CreateLogicsNodeItem(const QString toolname)
{
    m_NodeView->NodeItemFactory(toolname, 1, 1);
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
