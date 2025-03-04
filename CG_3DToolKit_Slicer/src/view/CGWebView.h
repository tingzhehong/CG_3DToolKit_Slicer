#ifndef CGWEBVIEW_H
#define CGWEBVIEW_H

#include <CGBaseWidget.h>

class CGWebView : public CGBaseWidget
{
    Q_OBJECT

public:
    explicit CGWebView(QWidget *parent = nullptr);
    ~CGWebView();

public:
    void InitUi() override;
    void InitConnections() override;
    void ShowView();

private:
    QWidget *m_pBrowser;
};

#endif // CGWEBVIEW_H
