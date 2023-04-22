#ifndef CGDATATREEVIEW_H
#define CGDATATREEVIEW_H

#include <CGBaseTreeWidget.h>

class CGDataTreeView : public CGBaseTreeWidget
{
    Q_OBJECT

public:
    explicit CGDataTreeView(CGBaseTreeWidget *parent = nullptr);
    ~CGDataTreeView();

signals:

public:
    void InitUi() override;
    void InitConnections() override;

public:
    QTreeWidget *m_DataTree;

};

#endif // CGDATATREEVIEW_H
