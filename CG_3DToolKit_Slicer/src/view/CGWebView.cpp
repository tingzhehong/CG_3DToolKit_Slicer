#include "CGWebView.h"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>
#include <QDebug>


CGWebView::CGWebView(QWidget *parent) : CGBaseWidget(parent), m_pBrowser(new QWidget)
{
    InitUi();
    InitConnections();
    setBackgroundRole(QPalette::Window);
    setWindowTitle(tr(u8"浏览窗口"));
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
}

CGWebView::~CGWebView()
{

}

void CGWebView::InitUi()
{
    m_pBrowser->setStyleSheet("background-color:rgb(99, 99, 99)");

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_pBrowser);
    setLayout(pMainLayout);
}

void CGWebView::InitConnections()
{

}
