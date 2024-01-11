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
    resize(1060, 600);

    font.setPointSize(10);
    pe.setColor(QPalette::WindowText,Qt::blue);

    m_StatusLb = new QLabel("", this);
    m_StatusLb->setFont(font);
    m_StatusLb->setPalette(pe);
    
    m_pOKBtn = new QPushButton(tr(u8"确定"), this);
    m_pOKBtn->setFixedSize(QSize(100, 36));
    m_pCancelBtn = new QPushButton(tr(u8"取消"), this);
    m_pCancelBtn->setFixedSize(QSize(100, 36));

    QHBoxLayout *pFristLayout = new QHBoxLayout();
    pFristLayout->addWidget(NodeBlockWidget::getInstance());

    QHBoxLayout *pSecondLayout = new QHBoxLayout();
    pSecondLayout->addWidget(m_StatusLb);
    pSecondLayout->addStretch();
    pSecondLayout->addWidget(m_pOKBtn);
    pSecondLayout->addWidget(m_pCancelBtn);
    pSecondLayout->addSpacing(15);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addLayout(pFristLayout);
    pMainLayout->addLayout(pSecondLayout);
    setLayout(pMainLayout);
}

void CGAlgorithmArgumentsDialog::InitConnections()
{
    connect(m_pOKBtn, &QPushButton::clicked, this, [&]{ NodeBlockWidget::getInstance()->RemoveShapeItem(); CGAlgorithmArgumentsDialog::OnOK(); });
    connect(m_pCancelBtn, &QPushButton::clicked, this, [&]{ NodeBlockWidget::getInstance()->RemoveShapeItem(); CGAlgorithmArgumentsDialog::OnCancel(); });
    connect(NodeBlockWidget::getInstance(), &NodeBlockWidget::SignalShapeItemValue, this, &CGAlgorithmArgumentsDialog::OnShapeItemValue);
}

void CGAlgorithmArgumentsDialog::OnOK()
{
    emit SignalSetArguments();
    this->accept();
}

void CGAlgorithmArgumentsDialog::OnCancel()
{
    this->reject();
    this->close();
}

void CGAlgorithmArgumentsDialog::OnShapeItemValue(const QString msg)
{
    m_StatusLb->setText(msg);
}

void CGAlgorithmArgumentsDialog::closeEvent(QCloseEvent *closeEvent)
{
    closeEvent->ignore();
}
