#include "CGWaitingDialog.h"
#include <QIcon>
#include <QLabel>
#include <QProgressBar>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

CGWaitingDialog::CGWaitingDialog(QWidget *parent): QDialog(parent)
{
    InitUi();
}

void CGWaitingDialog::InitUi()
{
    setWindowTitle(tr(u8"信息"));
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
    resize(500, 200);

    m_pWaitingMsgLb = new QLabel(tr(u8"正在保存数据文件......"), this);
    m_pProgressBar = new QProgressBar(this);

    QFont font;
    font.setPointSize(12);           // 设置字号
    m_pWaitingMsgLb->setFont(font);  // 设置字体

    m_pProgressBar->setOrientation(Qt::Horizontal);
    m_pProgressBar->setAlignment(Qt::AlignCenter);
    m_pProgressBar->setFixedHeight(30);
    m_pProgressBar->setMinimum(0);
    m_pProgressBar->setMaximum(100);
    m_pProgressBar->setValue(100);

    QHBoxLayout *pFristLayout = new QHBoxLayout();
    pFristLayout->addStretch();
    pFristLayout->addWidget(m_pWaitingMsgLb);
    pFristLayout->addStretch();

    QHBoxLayout *pSecondLayout = new QHBoxLayout();
    pSecondLayout->addSpacing(20);
    pSecondLayout->addWidget(m_pProgressBar);
    pSecondLayout->addSpacing(20);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addStretch();
    pMainLayout->addLayout(pFristLayout);
    pMainLayout->addStretch();
    pMainLayout->addLayout(pSecondLayout);
    pMainLayout->addStretch();

    setLayout(pMainLayout);
}
