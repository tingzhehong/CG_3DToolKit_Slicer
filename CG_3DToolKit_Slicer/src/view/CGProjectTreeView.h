#ifndef CGPROJECTTREEVIEW_H
#define CGPROJECTTREEVIEW_H

#include <CGBaseTreeWidget.h>

class CGProjectTreeView : public CGBaseTreeWidget
{
    Q_OBJECT

public:
    explicit CGProjectTreeView(CGBaseTreeWidget *parent = nullptr);
    ~CGProjectTreeView();

signals:

public:
    void InitUi() override;
    void InitConnections() override;

public:
    QTreeWidget *m_ProjectTree;
    QTreeWidgetItem *m_Window2D;
    QTreeWidgetItem *m_Window3D;
    QTreeWidgetItem *m_WindowNodeEdit;
    QTreeWidgetItem *m_WindowProfile;

    QStringList TreeItemNames{u8"2D  图像", u8"3D  图像", u8"流程节点", u8"轮廓分析"};

public:
    enum ProjectTreeItem
    {
        Window2D,
        Window3D,
        WindowNodeEdit,
        WindowProfile
    };

};

#endif // CGPROJECTTREEVIEW_H
