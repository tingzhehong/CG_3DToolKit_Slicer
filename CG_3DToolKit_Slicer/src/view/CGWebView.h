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

private:

};

#endif // CGWEBVIEW_H
