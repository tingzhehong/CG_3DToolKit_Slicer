#ifndef CGBASETREEWIDGET_H
#define CGBASETREEWIDGET_H

#include <QWidget>
#include <QDebug>

class QTreeWidget;
class QTreeWidgetItem;

class CGBaseTreeWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CGBaseTreeWidget(QWidget *parent = nullptr);
    ~CGBaseTreeWidget() = default;

public:
    virtual void InitUi() = 0;
    virtual void InitConnections() = 0;

};

#endif // CGBASETREEWIDGET_H
