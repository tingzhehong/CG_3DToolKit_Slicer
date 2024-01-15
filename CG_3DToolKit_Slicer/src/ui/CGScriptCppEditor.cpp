#include "CGScriptCppEditor.h"
#include <NodeBlock.h>
#include <QLabel>
#include <QLineEdit>
#include <QTextEdit>
#include <QPushButton>
#include <QComboBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QHeaderView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QMessageBox>
#include <QColor>
#include <QColorDialog>
#include <QFontDialog>
#include <QDebug>

CGScriptCppEditor::CGScriptCppEditor(QWidget *parent): QDialog(parent), m_IOColors(6)
{
    InitUi();
    InitTableWidget();
    InitFucntionstTree();
    InitConnections();
}

void CGScriptCppEditor::InitUi()
{
    setWindowTitle(tr(u8"脚本编辑器"));
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
    resize(1080, 600);

    m_pTableWidget = new QTableWidget();
    m_pTextEdit = new QTextEdit();
    m_pTextEdit->setLineWrapMode(QTextEdit::NoWrap);

    m_pFucntionsTree = new QTreeWidget(this);
    m_pFucntionsTree->setHeaderLabel(tr(u8"脚本函数"));
    m_pFucntionsTree->setFixedWidth(280);

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
    m_pFontSetBtn = new QPushButton(tr(u8"字体"), this);
    m_pColorSetBtn = new QPushButton(tr(u8"颜色"), this);

    QHBoxLayout *pFirstItemSetLayout = new QHBoxLayout();
    pFirstItemSetLayout->addStretch();
    pFirstItemSetLayout->addWidget(m_pItemSetBtn);
    pFirstItemSetLayout->addStretch();

    QVBoxLayout *pFirstLayout = new QVBoxLayout();
    pFirstLayout->addWidget(m_pTableWidget);
    pFirstLayout->addLayout(pFirstItemSetLayout);

    QHBoxLayout *pScriptLayout = new QHBoxLayout();
    pScriptLayout->addWidget(m_pTextEdit);
    pScriptLayout->addWidget(m_pFucntionsTree);

    QHBoxLayout *pSecondScriptSetLayout = new QHBoxLayout();
    pSecondScriptSetLayout->addStretch(2);
    pSecondScriptSetLayout->addWidget(m_pScriptSetBtn);
    pSecondScriptSetLayout->addStretch(1);
    pSecondScriptSetLayout->addWidget(m_pFontSetBtn);
    pSecondScriptSetLayout->addWidget(m_pColorSetBtn);
    pSecondScriptSetLayout->addSpacing(285);

    QVBoxLayout *pSecondLayout = new QVBoxLayout();
    pSecondLayout->addLayout(pScriptLayout);
    pSecondLayout->addLayout(pSecondScriptSetLayout);

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addLayout(pFirstLayout, 0, 0);
    pMainLayout->addLayout(pSecondLayout, 0, 1);
    pMainLayout->setColumnStretch(0, 3);
    pMainLayout->setColumnStretch(1, 7);
    setLayout(pMainLayout);
}

void CGScriptCppEditor::InitConnections()
{
    connect(m_pItemSetBtn, &QPushButton::clicked, this, &CGScriptCppEditor::OnItemSet);
    connect(m_pScriptSetBtn, &QPushButton::clicked, this, &CGScriptCppEditor::OnScriptSet);
    connect(m_pFontSetBtn, &QPushButton::clicked, this, &CGScriptCppEditor::OnFontSet);
    connect(m_pColorSetBtn, &QPushButton::clicked, this, &CGScriptCppEditor::OnColorSet);

    connect(m_pFucntionsTree, &QTreeWidget::itemDoubleClicked, this, [&](QTreeWidgetItem *item, int column){m_pTextEdit->insertPlainText(item->text(column) + ";");});
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

void CGScriptCppEditor::OnFontSet()
{
    QFont initial("微软雅黑", 10);
    QFont font = QFontDialog::getFont(0, initial, this, tr(u8"字体设置"));
    m_pTextEdit->setFont(font);
}

void CGScriptCppEditor::OnColorSet()
{
    QColor initial;
    initial.setRgbF(0, 0, 0);
    QColor color = QColorDialog::getColor(initial, this, tr(u8"颜色设置"));
    m_pTextEdit->setTextColor(color);
}

void CGScriptCppEditor::InitTableWidget()
{
    m_pTableWidget->setShowGrid(true);
    m_pTableWidget->verticalHeader()->setVisible(false);
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

void CGScriptCppEditor::InitFucntionstTree()
{
    QTreeWidgetItem *pScriptTopItem = new QTreeWidgetItem(QStringList{tr(u8"函数方法")});

    QTreeWidgetItem *pMathsItem = new QTreeWidgetItem(QStringList{tr(u8"算术")});
    QStringList maths;
    maths << "ScriptAdd(a, b)"
          << "ScriptSub(a, b)"
          << "ScriptMul(a, b)"
          << "ScriptDiv(a, b)";
    for (int i = 0; i < maths.count(); ++i) {
        QTreeWidgetItem *pMathsChildItem = new QTreeWidgetItem(QStringList() << maths.at(i));
        pMathsItem->addChild(pMathsChildItem);
    }

    QTreeWidgetItem *pFilterItem = new QTreeWidgetItem(QStringList{tr(u8"滤波")});
    QStringList filter;
    filter << "ScriptGaussianFilter(img, size)"
           << "ScriptMeanFilter(img, size, size)"
           << "ScriptMedianFilter(img, size, size)"
           << "ScriptVoxelFilter(cloud, size)";
    for (int j = 0; j < maths.count(); ++j) {
        QTreeWidgetItem *pFilterChildItem = new QTreeWidgetItem(QStringList() << filter.at(j));
        pFilterItem->addChild(pFilterChildItem);
    }

    pScriptTopItem->addChild(pMathsItem);
    pScriptTopItem->addChild(pFilterItem);
    m_pFucntionsTree->addTopLevelItem(pScriptTopItem);
    m_pFucntionsTree->expandItem(pScriptTopItem);
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
