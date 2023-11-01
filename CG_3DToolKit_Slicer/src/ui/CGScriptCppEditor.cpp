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
#include <QMessageBox>
#include <QColor>

CGScriptCppEditor::CGScriptCppEditor(QWidget *parent): QDialog(parent), m_IOColors(6)
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
    m_Input_1->addItems(QStringList() << tr("null") << tr(u8"2D 图像") << tr(u8"3D 点云") << tr(u8"numeric 数值"));
    m_Input_2 = new QComboBox();
    m_Input_2->addItems(QStringList() << tr("null") << tr(u8"2D 图像") << tr(u8"3D 点云") << tr(u8"numeric 数值"));
    m_Input_3 = new QComboBox();
    m_Input_3->addItems(QStringList() << tr("null") << tr(u8"2D 图像") << tr(u8"3D 点云") << tr(u8"numeric 数值"));

    m_Output_1 = new QComboBox();
    m_Output_1->addItems(QStringList() << tr("null") << tr(u8"2D 图像") << tr(u8"3D 点云") << tr(u8"numeric 数值"));
    m_Output_2 = new QComboBox();
    m_Output_2->addItems(QStringList() << tr("null") << tr(u8"2D 图像") << tr(u8"3D 点云") << tr(u8"numeric 数值"));
    m_Output_3 = new QComboBox();
    m_Output_3->addItems(QStringList() << tr("null") << tr(u8"2D 图像") << tr(u8"3D 点云") << tr(u8"numeric 数值"));

    QFont font;
    font.setPixelSize(14);
    font.setBold(true);
    m_Input_1->setFont(font);
    m_Input_2->setFont(font);
    m_Input_3->setFont(font);
    m_Output_1->setFont(font);
    m_Output_2->setFont(font);
    m_Output_3->setFont(font);

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

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addLayout(pFirstLayout, 0, 0);
    pMainLayout->addLayout(pSecondLayout, 0, 1);
    pMainLayout->setColumnStretch(0, 4);
    pMainLayout->setColumnStretch(1, 6);
    setLayout(pMainLayout);
}

void CGScriptCppEditor::InitConnections()
{
    connect(m_pItemSetBtn, &QPushButton::clicked, this, &CGScriptCppEditor::OnItemSet);
    connect(m_pScriptSetBtn, &QPushButton::clicked, this, &CGScriptCppEditor::OnScriptSet);
}

void CGScriptCppEditor::SetCurrentNodeBlock(NodeBlock *nodeblock)
{
    m_pNodeBlock = nodeblock;
}

NodeBlock *CGScriptCppEditor::GetCurrentNodeBlock() const
{
    return m_pNodeBlock;
}

void CGScriptCppEditor::OnItemSet()
{
    QColor Color_Input_1 = CheckItemIOColor(m_Input_1);
    QColor Color_Input_2 = CheckItemIOColor(m_Input_2);
    QColor Color_Input_3 = CheckItemIOColor(m_Input_3);

    QColor Color_Output_1 = CheckItemIOColor(m_Output_1);
    QColor Color_Output_2 = CheckItemIOColor(m_Output_2);
    QColor Color_Output_3 = CheckItemIOColor(m_Output_3);

    ClearItemPort();

    if (Color_Input_1 != Qt::white)
        m_pNodeBlock->m_NodeItem->createPortIn(8, Color_Input_1);
    if (Color_Input_2 != Qt::white)
        m_pNodeBlock->m_NodeItem->createPortIn(24, Color_Input_2);
    if (Color_Input_3 != Qt::white)
        m_pNodeBlock->m_NodeItem->createPortIn(40, Color_Input_3);

    if (Color_Output_1 != Qt::white)
        m_pNodeBlock->m_NodeItem->createPortOut(8, Color_Output_1);
    if (Color_Output_2 != Qt::white)
        m_pNodeBlock->m_NodeItem->createPortOut(24, Color_Output_2);
    if (Color_Output_3 != Qt::white)
        m_pNodeBlock->m_NodeItem->createPortOut(40, Color_Output_3);

    m_IOColors[0] = Color_Input_1;
    m_IOColors[1] = Color_Input_2;
    m_IOColors[2] = Color_Input_3;
    m_IOColors[3] = Color_Output_1;
    m_IOColors[4] = Color_Output_2;
    m_IOColors[5] = Color_Output_3;

    QString IOColors;
    for (int i = 0; i < m_IOColors.size(); ++i)
    {
        QString color = m_IOColors[i].name();
        IOColors.append(color).append(",");
    }
    m_pNodeBlock->m_NodeItem->m_Parameters[u8"代码"] = m_pTextEdit->toPlainText();
    m_pNodeBlock->m_NodeItem->m_Parameters[u8"端口"] = IOColors;

    int ret = QMessageBox::information(this, tr(u8"脚本编辑器"), tr(u8"脚本端口设置完成！"), QMessageBox::Accepted);
    if (ret == QMessageBox::Accepted)
        this->close();
}

void CGScriptCppEditor::OnScriptSet()
{
    m_pNodeBlock->m_NodeItem->m_Parameters[u8"代码"] = m_pTextEdit->toPlainText();

    int ret = QMessageBox::information(this, tr(u8"脚本编辑器"), tr(u8"脚本代码输入完成！"), QMessageBox::Accepted);
    if (ret == QMessageBox::Accepted)
        this->close();
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

void CGScriptCppEditor::ClearItemPort()
{
    foreach (PortItem *port, m_pNodeBlock->m_NodeItem->portList()) {
        m_pNodeBlock->m_NodeItem->removePort(port);
    }
}

QColor CGScriptCppEditor::CheckItemIOColor(QComboBox *combox)
{
    QColor clr = QColor(Qt::white);

    switch (combox->currentIndex())
    {
    case 1:
        clr = QColor(Qt::yellow);
        break;
    case 2:
        clr = QColor(Qt::red);
        break;
    case 3:
        clr = QColor(Qt::cyan);
        break;
    default:
        clr = QColor(Qt::white);
        break;
    }

    return clr;
}
