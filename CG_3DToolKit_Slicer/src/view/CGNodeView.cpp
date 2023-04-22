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
    Test();

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_NodeView);

    setLayout(pMainLayout);
    setVisible(true);
}

void CGNodeView::InitConnections()
{

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
