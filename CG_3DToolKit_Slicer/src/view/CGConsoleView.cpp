#include "CGConsoleView.h"
#include <QListWidget>
#include <QFont>
#include <QLabel>
#include <QMessageBox>
#include <QDateTime>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QMenu>
#include <QAction>

CGConsoleView *CGConsoleView::m_CGConsoleView = nullptr;

CGConsoleView::CGConsoleView(CGBaseWidget *parent) : CGBaseWidget(parent)
{
    InitUi();
    InitConnections();
}

void CGConsoleView::OnConsoleClear()
{
    m_pListWidget->clear();
    m_pListWidget->update();
}

CGConsoleView *CGConsoleView::getInstance()
{
    if (!m_CGConsoleView)
    {
        m_CGConsoleView = new CGConsoleView();
    }
    return m_CGConsoleView;
}

void CGConsoleView::InitUi()
{
    QFont font;
    font.setBold(true);
    m_pListWidget = new QListWidget(this);
    m_pListWidget->setFixedHeight(100);
    m_pListWidget->setStyleSheet("background-color:rgb(255, 255, 255)");

    QVBoxLayout *pConsoleLayout = new QVBoxLayout();
    pConsoleLayout->addWidget(m_pListWidget);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addLayout(pConsoleLayout);

    setLayout(pMainLayout);
    setVisible(true);

    m_action = new QAction(tr(u8"清除日志"));
    m_pListWidget->addAction(m_action);
    m_pListWidget->setContextMenuPolicy(Qt::ActionsContextMenu);
}

void CGConsoleView::InitConnections()
{
    connect(m_action, &QAction::triggered, this, &CGConsoleView::OnConsoleClear);
}

void CGConsoleView::ConsoleOut(const QString msg)
{
    QDateTime CurrentDataTime = QDateTime::currentDateTime();
    QString current_date_time = "[" + CurrentDataTime.toString("yyyy/MM/dd hh:mm:ss:zzz") + "]";
    QString str;
    str.append(current_date_time);
    str.append(" : ");
    str.append(msg);

    QListWidgetItem *item = new QListWidgetItem(str);
    m_pListWidget->insertItem(0, item);

    int num = m_pListWidget->count();
    if (num >= 99) {
        m_pListWidget->clear();
    }
}
