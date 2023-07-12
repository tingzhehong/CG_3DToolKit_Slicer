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
    void Signal2DToolClear();
    void Signal3DToolClear();
    void SignalProfileToolClear();

public slots:
    void OnAlgorithmPluginAdd(QPair<QStringList, QStringList> names);

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
    QStringList m_3DToolNames{u8"距离", u8"角度", u8"包围盒", u8"平面"};
    QStringList m_ProfileToolNames{u8"2点指定", u8"垂直线", u8"水平线", u8"矩形", u8"圆", u8"圆弧", u8"2点指定剖面", u8"垂直剖面", u8"水平剖面"};
    QStringList m_MathsNames{u8"加", u8"减", u8"乘", u8"除", u8"数值/输入", u8"数值/输出"};
    QStringList m_LogicsNames{u8"条件", u8"循环", u8"组"};
    QStringList m_2DFuctionNames{u8"2D数据源", u8"2D数据终端"};
    QStringList m_3DFuctionNames{u8"3D数据源", u8"3D数据终端"};

    QStringList m_DataTreeNames{u8"2D工具箱", u8"3D工具箱", u8"轮廓工具箱", u8"数学计算", u8"逻辑运算", u8"2D功能算子", u8"3D功能算子"};

    enum DateTreeEnum
    {
        ToolBox2D,
        ToolBox3D,
        ToolBoxProfile,
        Maths,
        Logics,
        Fuction2D,
        Fuction3D
    };

private:
    QAction *m_action2DToolClear;
    QAction *m_action3DToolClear;
    QAction *m_actionProfileToolClear;

};

#endif // CGDATATREEVIEW_H
