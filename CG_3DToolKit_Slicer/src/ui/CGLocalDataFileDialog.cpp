#include "CGLocalDataFileDialog.h"
#include <NodeBlock.h>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QDebug>

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
    connect(m_pFileBtn, &QPushButton::clicked, this, [&](){switch (m_2d3dfile) { case 2: OnLoadImage(); break; case 3: OnLoadPointCloud(); break; default: break; }});
    connect(m_pOKBtn, &QPushButton::clicked, [this]{m_pNodeBlock->m_NodeItem->m_Parameters[u8"文件"] = m_FileName; accept();});
}

void CGLocalDataFileDialog::SetCurrentNodeBlock(NodeBlock *nodeblock)
{
    m_pNodeBlock = nodeblock;
}

NodeBlock *CGLocalDataFileDialog::GetCurrentNodeBlock() const
{
    return m_pNodeBlock;
}

void CGLocalDataFileDialog::OnLoadImage()
{
    QString FileName = QFileDialog::getOpenFileName(this, tr(u8"打开图像文件"), ".", "*.bmp *.png *.jpg *.jpeg *.tif *.tiff");

    if (FileName.isEmpty())
    {
        QMessageBox::information(this, tr(u8"信息"), tr(u8"请选择图像文件！"));
    }
    else
    {
        m_pFilePath->setText(FileName);
        m_FileName = FileName;
    }
}

void CGLocalDataFileDialog::OnLoadPointCloud()
{
    QString FileName = QFileDialog::getOpenFileName(this, tr(u8"打开点云文件"), ".", "*.pcd");

    if (FileName.isEmpty())
    {
        QMessageBox::information(this, tr(u8"信息"), tr(u8"请选择点云文件！"));
    }
    else
    {
        m_pFilePath->setText(FileName);
        m_FileName = FileName;
    }
}
