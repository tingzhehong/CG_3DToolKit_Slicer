#include "CGScriptCppEditor.h"
#include <NodeBlock.h>
#include <QLabel>
#include <QLineEdit>
#include <QTextEdit>
#include <QPushButton>
#include <QComboBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

CGScriptCppEditor::CGScriptCppEditor(QWidget *parent): QDialog(parent)
{
    InitUi();
    InitConnections();
    InitTableWidget();
}

void CGScriptCppEditor::InitUi()
{
    setWindowTitle(tr(u8"脚本编辑器"));
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
    resize(800, 600);

    m_pTableWidget = new QTableWidget();
    m_pTextEdit = new QTextEdit();

    m_Input_1 = new QComboBox();
    m_Input_1->addItems(QStringList() << tr("NULL") << tr(u8"2D 图像") << tr(u8"3D 点云") << tr(u8"numeric 数值"));
    m_Input_2 = new QComboBox();
    m_Input_2->addItems(QStringList() << tr("NULL") << tr(u8"2D 图像") << tr(u8"3D 点云") << tr(u8"numeric 数值"));
    m_Input_3 = new QComboBox();
    m_Input_3->addItems(QStringList() << tr("NULL") << tr(u8"2D 图像") << tr(u8"3D 点云") << tr(u8"numeric 数值"));

    m_Output_1 = new QComboBox();
    m_Output_1->addItems(QStringList() << tr("NULL") << tr(u8"2D 图像") << tr(u8"3D 点云") << tr(u8"numeric 数值"));
    m_Output_2 = new QComboBox();
    m_Output_2->addItems(QStringList() << tr("NULL") << tr(u8"2D 图像") << tr(u8"3D 点云") << tr(u8"numeric 数值"));
    m_Output_3 = new QComboBox();
    m_Output_3->addItems(QStringList() << tr("NULL") << tr(u8"2D 图像") << tr(u8"3D 点云") << tr(u8"numeric 数值"));

    m_pItemSetBtn = new QPushButton(tr(u8"确定"), this);
    m_pScriptSetBtn = new QPushButton(tr(u8"确定"), this);

    QHBoxLayout *pFirstItemSetLayout = new QHBoxLayout();
    pFirstItemSetLayout->addStretch();
    pFirstItemSetLayout->addWidget(m_pItemSetBtn);
    pFirstItemSetLayout->addStretch();

    QVBoxLayout *pFirstLayout = new QVBoxLayout();
    pFirstLayout->addWidget(m_pTableWidget);
    pFirstLayout->addLayout(pFirstItemSetLayout);

    QHBoxLayout *pSecondScriptSetLayout = new QHBoxLayout();
    pSecondScriptSetLayout->addStretch();
    pSecondScriptSetLayout->addWidget(m_pScriptSetBtn);
    pSecondScriptSetLayout->addStretch();

    QVBoxLayout *pSecondLayout = new QVBoxLayout();
    pSecondLayout->addWidget(m_pTextEdit);
    pSecondLayout->addLayout(pSecondScriptSetLayout);

    QHBoxLayout *pMainLayout = new QHBoxLayout();
    pMainLayout->addLayout(pFirstLayout);
    pMainLayout->addLayout(pSecondLayout);
    setLayout(pMainLayout);
}

void CGScriptCppEditor::InitConnections()
{

}

void CGScriptCppEditor::SetCurrentNodeBlock(NodeBlock *nodeblock)
{
    m_pNodeBlock = nodeblock;
}

NodeBlock *CGScriptCppEditor::GetCurrentNodeBlock() const
{
    return m_pNodeBlock;
}

void CGScriptCppEditor::InitTableWidget()
{
    m_pTableWidget->setShowGrid(true);
    m_pTableWidget->verticalHeader()->setVisible(true);
    m_pTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_pTableWidget->setColumnCount(3);
    m_pTableWidget->setRowCount(3);
    m_pTableWidget->setColumnWidth(0, 50);

    m_pTableWidget->setHorizontalHeaderLabels(QStringList() << QString::fromLocal8Bit("端口")
                                                            << QString::fromLocal8Bit("输入")
                                                            << QString::fromLocal8Bit("输出"));

    m_pTableWidget->setItem(0, 0, new QTableWidgetItem(QString("  1  ")));
    m_pTableWidget->setItem(1, 0, new QTableWidgetItem(QString("  2  ")));
    m_pTableWidget->setItem(2, 0, new QTableWidgetItem(QString("  3  ")));
    m_pTableWidget->setCellWidget(0, 1, m_Input_1);  m_pTableWidget->setCellWidget(0, 2, m_Output_1);
    m_pTableWidget->setCellWidget(1, 1, m_Input_2);  m_pTableWidget->setCellWidget(1, 2, m_Output_2);
    m_pTableWidget->setCellWidget(2, 1, m_Input_3);  m_pTableWidget->setCellWidget(2, 2, m_Output_3);
}
