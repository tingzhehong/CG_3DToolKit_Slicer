#include "CGValueIndicator.h"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QTextEdit>
#include <QPushButton>
#include <QIcon>
#include <QDebug>

CGValueIndicator::CGValueIndicator(QWidget *parent): QDialog(parent)
{
    InitUi();
    InitConnections();
}

void CGValueIndicator::InitUi()
{
    setWindowTitle(tr(u8"数值显示器"));
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
    resize(600, 500);

    m_pTextEdit = new QTextEdit();
    m_pTextEdit->setLineWrapMode(QTextEdit::NoWrap);
    m_pTextEdit->setFontPointSize(12);

    m_pCloseBtn = new QPushButton(tr(u8"关闭"), this);

    QGridLayout *pTextLayout = new QGridLayout();
    pTextLayout->addWidget(m_pTextEdit);

    QGroupBox *pTextGroupBox = new QGroupBox(tr(u8"Slicer Json"));
    pTextGroupBox->setLayout(pTextLayout);

    QHBoxLayout *pLastLayout = new QHBoxLayout();
    pLastLayout->addStretch();
    pLastLayout->addWidget(m_pCloseBtn);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addWidget(pTextGroupBox);
    pMainLayout->addLayout(pLastLayout);

    setLayout(pMainLayout);
}

void CGValueIndicator::InitConnections()
{
    connect(m_pCloseBtn, &QPushButton::clicked, [this]{this->close();});
}

void CGValueIndicator::LoadJsonValueText(const QString str)
{
    m_pTextEdit->setText(str);
}
