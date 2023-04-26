#include "CGDepthImageDialog.h"
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QPushButton>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>


CGDepthImageDialog::CGDepthImageDialog(QWidget *parent): QDialog(parent)
{
    setWindowTitle(tr(u8"深度图像设置"));
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
    resize(360, 200);
    InitUi();
    InitConnections();
}

void CGDepthImageDialog::InitUi()
{
    m_TipLB = new QLabel(tr(u8"Tip：深度图像须设置图像像素单量！"), this);
    m_xPitchLB = new QLabel(tr(u8"X Pitch: "));
    m_yPitchLB = new QLabel(tr(u8"Y Pitch: "));
    m_UpLimitLB = new QLabel(tr(u8"上限值: "));
    m_DownLimitLB = new QLabel(tr(u8"下限值: "));

    m_xPitchLE = new QLineEdit("0", this);
    m_xPitchLE->setFixedSize(QSize(80, 20));
    m_yPitchLE = new QLineEdit("0", this);
    m_yPitchLE->setFixedSize(QSize(80, 20));
    m_UpLimitLE = new QLineEdit("99", this);
    m_UpLimitLE->setFixedSize(QSize(80, 20));
    m_DownLimitLE = new QLineEdit("-99", this);
    m_DownLimitLE->setFixedSize(QSize(80, 20));

    m_OKBtn = new QPushButton(tr(u8"确定"), this);
    m_CancelBtn = new QPushButton(tr(u8"取消"), this);

    QHBoxLayout *pFirstLayout = new QHBoxLayout();
    pFirstLayout->addSpacing(10);
    pFirstLayout->addWidget(m_TipLB);

    QHBoxLayout *pSecondLayout = new QHBoxLayout();
    pSecondLayout->addStretch();
    pSecondLayout->addWidget(m_xPitchLB);
    pSecondLayout->addWidget(m_xPitchLE);
    pSecondLayout->addStretch();
    pSecondLayout->addWidget(m_UpLimitLB);
    pSecondLayout->addWidget(m_UpLimitLE);
    pSecondLayout->addStretch();

    QHBoxLayout *pThirdLayout = new QHBoxLayout();
    pThirdLayout->addStretch();
    pThirdLayout->addWidget(m_yPitchLB);
    pThirdLayout->addWidget(m_yPitchLE);
    pThirdLayout->addStretch();
    pThirdLayout->addWidget(m_DownLimitLB);
    pThirdLayout->addWidget(m_DownLimitLE);
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

void CGDepthImageDialog::InitConnections()
{
    connect(m_OKBtn, &QPushButton::clicked, [this](){
        xPitch = m_xPitchLE->text().trimmed().toFloat();
        yPitch = m_yPitchLE->text().trimmed().toFloat();
        upLimit = m_UpLimitLE->text().trimmed().toFloat();
        downLimit = m_DownLimitLE->text().trimmed().toFloat();
        accept();
    });

    connect(m_CancelBtn, &QPushButton::clicked, [this](){
        reject();
    });
}
