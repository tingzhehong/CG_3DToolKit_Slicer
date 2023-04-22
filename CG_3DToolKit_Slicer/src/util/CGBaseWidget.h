#ifndef CGBASEWIDGET_H
#define CGBASEWIDGET_H

#include <QWidget>
#include <QDebug>

class QLabel;
class QLineEdit;
class QPushButton;

class CGBaseWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CGBaseWidget(QWidget *parent = nullptr);
    ~CGBaseWidget() = default;

public:
    virtual void InitUi() = 0;
    virtual void InitConnections() = 0;

};

#endif // CGBASEWIDGET_H
