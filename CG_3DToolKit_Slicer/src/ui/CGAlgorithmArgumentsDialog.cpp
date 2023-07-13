#include "CGAlgorithmArgumentsDialog.h"
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QCloseEvent>


CGAlgorithmArgumentsDialog::CGAlgorithmArgumentsDialog(QWidget *parent) : QDialog(parent)
{
    InitUi();
    InitConnections();

}

void CGAlgorithmArgumentsDialog::InitUi()
{
    setWindowTitle(tr(u8"功能算子  参数设置"));
    setWindowFlags(Qt::WindowTitleHint | Qt::CustomizeWindowHint);
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
    resize(960, 600);

    m_pOKBtn = new QPushButton(tr(u8"确定"), this);
    m_pOKBtn->setFixedSize(QSize(100, 36));
    m_pCancelBtn = new QPushButton(tr(u8"取消"), this);
    m_pCancelBtn->setFixedSize(QSize(100, 36));

    QHBoxLayout *pFristLayout = new QHBoxLayout();
    pFristLayout->addWidget(NodeBlockWidget::getInstance());

    QHBoxLayout *pSecondLayout = new QHBoxLayout();
    pSecondLayout->addStretch();
    pSecondLayout->addWidget(m_pOKBtn);
    pSecondLayout->addWidget(m_pCancelBtn);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addLayout(pFristLayout);
    pMainLayout->addLayout(pSecondLayout);
    setLayout(pMainLayout);
}

void CGAlgorithmArgumentsDialog::InitConnections()
{
    connect(m_pOKBtn, &QPushButton::clicked, this, &CGAlgorithmArgumentsDialog::OnOK);
    connect(m_pCancelBtn, &QPushButton::clicked, this, &CGAlgorithmArgumentsDialog::OnCancel);
}

void CGAlgorithmArgumentsDialog::OnOK()
{
    this->accept();
}

void CGAlgorithmArgumentsDialog::OnCancel()
{
    this->reject();
    this->close();
}

void CGAlgorithmArgumentsDialog::closeEvent(QCloseEvent *closeEvent)
{
    closeEvent->ignore();
}
