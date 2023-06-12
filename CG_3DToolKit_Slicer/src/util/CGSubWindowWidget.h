#ifndef CGSUBWINDOWWIDGET_H
#define CGSUBWINDOWWIDGET_H

#include <QMdiSubWindow>

class QHBoxLayout;
class QVBoxLayout;
class QGridLayout;

class CGSubWindowWidget : public QMdiSubWindow
{
    Q_OBJECT

public:
    explicit CGSubWindowWidget();
    ~CGSubWindowWidget() = default;

protected:
    void closeEvent(QCloseEvent *closeEvent);

};

#endif // CGSUBWINDOWWIDGET_H
