#include "CGDisOrderDialog.h"
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QPushButton>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>


CGDisOrderDialog::CGDisOrderDialog(QWidget *parent): QDialog(parent)
{

    setWindowTitle(tr(u8"无序点云设置"));
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
    resize(360, 200);
    InitUi();
    InitConnections();
}

void CGDisOrderDialog::InitUi()
{
    m_TipLB = new QLabel(tr(u8"Tip：无序点云须设置图像尺寸范围！"), this);
    m_WidthLB = new QLabel(tr(u8" Width: "), this);
    m_HeightLB = new QLabel(tr(u8"Heigth: "), this);

    m_WidthLE = new QLineEdit("0", this);
    m_WidthLE->setFixedWidth(80);
    m_WidthLE->setFixedHeight(20);
    m_HeightLE = new QLineEdit("0", this);
    m_HeightLE->setFixedWidth(80);
    m_HeightLE->setFixedHeight(20);

    m_OKBtn = new QPushButton(tr(u8"确定"), this);
    m_CancelBtn = new QPushButton(tr(u8"取消"), this);

    QHBoxLayout *pFirstLayout = new QHBoxLayout();
    pFirstLayout->addSpacing(10);
    pFirstLayout->addWidget(m_TipLB);

    QHBoxLayout *pSecondLayout = new QHBoxLayout();
    pSecondLayout->addStretch();
    pSecondLayout->addWidget(m_WidthLB);
    pSecondLayout->addWidget(m_WidthLE);
    pSecondLayout->addStretch();

    QHBoxLayout *pThirdLayout = new QHBoxLayout();
    pThirdLayout->addStretch();
    pThirdLayout->addWidget(m_HeightLB);
    pThirdLayout->addWidget(m_HeightLE);
    pThirdLayout->addStretch();

    QHBoxLayout *pLastLayout = new QHBoxLayout();
    pLastLayout->addStretch();
    pLastLayout->addWidget(m_OKBtn);
    pLastLayout->addWidget(m_CancelBtn);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addSpacing(10);
    pMainLayout->addLayout(pFirstLayout);
    pMainLayout->addSpacing(20);
    pMainLayout->addLayout(pSecondLayout);
    pMainLayout->addSpacing(10);
    pMainLayout->addLayout(pThirdLayout);
    pMainLayout->addStretch();
    pMainLayout->addLayout(pLastLayout);

    setLayout(pMainLayout);
}

void CGDisOrderDialog::InitConnections()
{
    connect(m_OKBtn, &QPushButton::clicked, [this](){
        m_Width = m_WidthLE->text().trimmed().toInt();
        m_Height = m_HeightLE->text().trimmed().toInt();
        accept();
    });

    connect(m_CancelBtn, &QPushButton::clicked, [this](){
        reject();
    });
}
