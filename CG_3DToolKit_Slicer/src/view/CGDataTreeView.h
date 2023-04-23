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

    QTreeWidgetItem *m_ToolBoxTopItem ;
    QTreeWidgetItem *m_NodeFlowTopItem;
    QTreeWidgetItem *m_FuctionTopItem;

    QTreeWidgetItem *m_2DToolBox;
    QTreeWidgetItem *m_3DToolBox;
    QTreeWidgetItem *m_ProfileToolBox;

    QTreeWidgetItem *m_Maths;
    QTreeWidgetItem *m_Logics;

    QTreeWidgetItem *m_2DFuction;
    QTreeWidgetItem *m_3DFuction;

    QStringList m_2DToolNames{u8"线段", u8"矩形", u8"圆", u8"圆弧"};
    QStringList m_3DToolNames{u8"距离", u8"角度", u8"包围盒"};
    QStringList m_MathsNames{u8"加", u8"减", u8"乘", u8"除"};
    QStringList m_LogicsNames{u8"条件", u8"循环"};
    QStringList m_ProfileToolNames{u8"2点指定", u8"垂直线", u8"水平线", u8"矩形", u8"圆", u8"圆弧"};

};

#endif // CGDATATREEVIEW_H
