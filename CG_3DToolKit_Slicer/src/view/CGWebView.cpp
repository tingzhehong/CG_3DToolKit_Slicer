#include "CGWebView.h"
#include "Windows.h"
#include <QWindow>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>
#include <QDebug>
#include <QProcess>
#include <QThread>


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

void CGWebView::ShowView()
{
    QProcess *process = new QProcess(this);
    process->start("C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe");

    if (!process->waitForStarted()) { qDebug() << u8"无法启动浏览器程序"; return; }

    WId winid = process->processId(); Q_UNUSED(winid);
    WId winId = (WId)FindWindow(L"Qt5152QWindowIcon", L"Browser");
    QWindow *window = QWindow::fromWinId(winId);
    window->setFlags(window->flags() | Qt::CustomizeWindowHint | Qt::WindowTitleHint);

    QWidget *widget;
    widget = QWidget::createWindowContainer(window, m_pBrowser, Qt::Widget);
    widget->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    widget->adjustSize();
    widget->setMinimumSize(800, 600);
    widget->show();
}
