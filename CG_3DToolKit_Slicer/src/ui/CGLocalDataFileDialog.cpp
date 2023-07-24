#include "CGLocalDataFileDialog.h"
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>

CGLocalDataFileDialog::CGLocalDataFileDialog(QWidget *parent): QDialog(parent)
{
    InitUi();
    InitConnections();
}

void CGLocalDataFileDialog::InitUi()
{
    setWindowTitle(tr(u8"本地算子  文件设置"));
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
    resize(500, 100);

    m_pLabel = new QLabel(tr(u8"文件："), this);
    m_pFilePath = new QLineEdit(this);
    m_pFilePath->setFixedHeight(30);
    m_pFileBtn = new QPushButton(tr(u8"载入"), this);
    m_pFileBtn->setFixedHeight(30);
    m_pOKBtn = new QPushButton(tr(u8"确定"), this);
    m_pOKBtn->setFixedHeight(30);

    QHBoxLayout *pMainLayout = new QHBoxLayout();
    pMainLayout->addWidget(m_pLabel);
    pMainLayout->addWidget(m_pFilePath);
    pMainLayout->addWidget(m_pFileBtn);
    pMainLayout->addWidget(m_pOKBtn);
    setLayout(pMainLayout);
}

void CGLocalDataFileDialog::InitConnections()
{

}
