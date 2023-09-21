#include "CGWebView.h"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>
#include <QDebug>


CGWebView::CGWebView(QWidget *parent) : CGBaseWidget(parent)
{
    InitUi();
    InitConnections();
    setWindowTitle(tr(u8"浏览窗口"));
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
}

CGWebView::~CGWebView()
{

}

void CGWebView::InitUi()
{
    setBackgroundRole(QPalette::Window);
}

void CGWebView::InitConnections()
{

}
