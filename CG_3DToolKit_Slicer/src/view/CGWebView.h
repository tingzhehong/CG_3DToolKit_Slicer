#ifndef CGWEBVIEW_H
#define CGWEBVIEW_H

#include <CGBaseWidget.h>
#include <QSharedPointer>
#include "miniblink.h"

class QLabel;
class QLineEdit;
class QPushButton;
class QVBoxLayout;
class CGWebView : public CGBaseWidget
{
    Q_OBJECT

public:
    explicit CGWebView(QWidget *parent = nullptr);
    ~CGWebView();

public:
    void InitUi() override;
    void InitConnections() override;
    void InitWebView();
    void ShowView();
    void DestroyView();

public:
    QWidget *m_pBrowser;
    QVBoxLayout *m_pMainLayout;

private:
    QLineEdit *m_url;

    QPushButton *m_forword;
    QPushButton *m_back;
    QPushButton *m_reload;
    QPushButton *m_stop;
    QPushButton *m_zoomin;
    QPushButton *m_zoomout;

    QString m_CurrentUrl;

public:
    QSharedPointer<miniblink> webView;
};

#endif // CGWEBVIEW_H
