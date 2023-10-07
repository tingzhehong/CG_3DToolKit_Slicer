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
    setBackgroundRole(QPalette::Window);
    setWindowTitle(tr(u8"浏览窗口"));
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
}

CGWebView::~CGWebView()
{

}

void CGWebView::InitUi()
{

}

void CGWebView::InitConnections()
{

}
