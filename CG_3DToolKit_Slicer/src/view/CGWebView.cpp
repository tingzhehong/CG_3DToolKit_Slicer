#include "CGWebView.h"
#include "Windows.h"
#include <QWindow>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>
#include <QDebug>


CGWebView::CGWebView(QWidget *parent) : CGBaseWidget(parent), m_pBrowser(new QWidget)
{
    InitUi();
    InitWebView();
    setBackgroundRole(QPalette::Window);
    setWindowTitle(tr(u8"浏览窗口"));
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
}

CGWebView::~CGWebView()
{

}

void CGWebView::InitUi()
{
    m_pBrowser->setStyleSheet("background-color:rgb(255, 255, 255)");
    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_pBrowser);
    setLayout(pMainLayout);
}

void CGWebView::InitConnections()
{
    connect(m_url, &QLineEdit::returnPressed, [&](){webView->load(m_url->text());});

    connect(m_forword, &QPushButton::clicked, [&](){webView->load(m_url->text());});
    connect(m_back, &QPushButton::clicked, [&](){webView->back();});
    connect(m_reload ,&QPushButton::clicked, [&](){webView->reload();});
    connect(m_stop, &QPushButton::clicked, [&](){webView->stop();});

    connect(m_zoomin, &QPushButton::clicked, [&](){
            float factor = webView->getZoom();
            if (factor < 5.0) webView->setZoom(factor + 0.1);
    });
    connect(m_zoomout, &QPushButton::clicked, [&](){
            float factor = webView->getZoom();
            if (factor > 0.25) webView->setZoom(factor - 0.1);
    });

    connect(webView.data(), &miniblink::loadFinished, [&](){m_CurrentUrl = webView->getURL();m_url->setText(m_CurrentUrl);});
}

void CGWebView::InitWebView()
{
    webView = QSharedPointer<miniblink>(new miniblink);

    m_url = new QLineEdit();
    m_url->setPlaceholderText("https://");
    m_url->setText("https://map.baidu.com");
    m_CurrentUrl = "https://map.baidu.com";

    m_forword = new QPushButton(tr(u8"前进"), this);
    m_back = new QPushButton(tr(u8"后退"), this);
    m_reload = new QPushButton(tr(u8"刷新"), this);
    m_stop = new QPushButton(tr(u8"停止"), this);
    m_zoomin = new QPushButton(tr(u8"放大"), this);
    m_zoomout = new QPushButton(tr(u8"缩小"), this);

    m_forword->setIcon(QIcon(":/res/icon/forward.png"));
    m_back->setIcon(QIcon(":/res/icon/backward.png"));
    m_reload->setIcon(QIcon(":/res/icon/recyle.png"));
    m_stop->setIcon(QIcon(":/res/icon/stop.png"));
    m_zoomin->setIcon(QIcon(":/res/icon/ccZoomIn.png"));
    m_zoomout->setIcon(QIcon(":/res/icon/ccZoomOut.png"));

    QHBoxLayout *topLayout = new QHBoxLayout;
    topLayout->addWidget(m_back);
    topLayout->addWidget(m_forword);
    topLayout->addWidget(m_reload);
    topLayout->addWidget(m_stop);
    topLayout->addWidget(m_url);
    topLayout->addWidget(m_zoomin);
    topLayout->addWidget(m_zoomout);

    m_pMainLayout = new QVBoxLayout();
    m_pMainLayout->addLayout(topLayout);

    m_pBrowser->setLayout(m_pMainLayout);
}

void CGWebView::ShowView()
{
    webView = QSharedPointer<miniblink>(new miniblink);
    webView->load(m_CurrentUrl);

    m_url->setText(m_CurrentUrl);
    m_pMainLayout->addWidget(webView.data());

    InitConnections();
}

void CGWebView::DestroyView()
{
    m_CurrentUrl = m_url->text();
    m_pMainLayout->removeWidget(webView.data());

    webView->destroy();
}
