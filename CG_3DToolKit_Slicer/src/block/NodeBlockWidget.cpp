#include "NodeBlockWidget.h"
#include <QLabel>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QDebug>
#include <CGVTKHeader.h>
#include <CGVTKUtils.h>
#include <CGVTKWidget.h>


NodeBlockWidget *NodeBlockWidget::m_NodeBlockWidget = nullptr;

NodeBlockWidget::NodeBlockWidget(QWidget *parent) : QWidget(parent)
{
    InitUi();
    InitConnections();
    InitTableWidget();
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
    m_ArgumentsTable = new QTableWidget(this);
    m_ArgumentsTable->setShowGrid(true);
    m_ArgumentsTable->verticalHeader()->setVisible(false);
    m_ArgumentsTable->setFixedWidth(360);

    m_CGVTKWidget = new CGVTKWidget(this);

    double clr[3];
    QColor defaultColor(25, 50, 75);
    VTKUtils::vtkColor(defaultColor, clr);
    m_CGVTKWidget->defaultRenderer()->SetBackground(clr);
    m_CGVTKWidget->showOrientationMarker();
    m_CGVTKWidget->update();

    QHBoxLayout *pArgumentsLayout = new QHBoxLayout();
    pArgumentsLayout->addWidget(m_ArgumentsTable);
    pArgumentsLayout->addWidget(m_CGVTKWidget);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addLayout(pArgumentsLayout);
    setLayout(pMainLayout);
}

void NodeBlockWidget::InitConnections()
{

}

void NodeBlockWidget::InitTableWidget()
{
    m_ArgumentsTable->setColumnCount(3);
    m_ArgumentsTable->setColumnWidth(0, 60);
    m_ArgumentsTable->setColumnWidth(1, 160);
    m_ArgumentsTable->setColumnWidth(2, 120);
    m_ArgumentsTable->setRowCount(16);
    m_ArgumentsTable->setHorizontalHeaderLabels(QStringList() << QString::fromLocal8Bit("序号")
                                                              << QString::fromLocal8Bit("参数")
                                                              << QString::fromLocal8Bit("数值"));
}
