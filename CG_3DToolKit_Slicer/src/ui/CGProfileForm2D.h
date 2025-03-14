﻿#ifndef CGPROFILEFORM2D_H
#define CGPROFILEFORM2D_H

#include <QWidget>
#include <CGGraphicsView.h>
#include <CGImage2DGraphicsItemAdapter.h>

class CGGraphicsLineItem;
class CGGraphicsRectItem;
class CGGraphicsCircleItem;
class CGGraphicsLineItemHorizontal;
class CGGraphicsLineItemVertical;

class CGProfileForm2D : public QWidget
{
    Q_OBJECT

public:
    explicit CGProfileForm2D(QWidget *parent = nullptr);
    ~CGProfileForm2D() = default;

public slots:
    void OnUseTool();
    void OnDelTool();

public:
    void InitUi();
    void InitConnections();
    void InitTools();
    void RemoveTools();

    void SetVerticalLine(double pos);
    void SetHorizontalLine(double pos);
    void SetTwoPointLine(double* pos_1, double* pos_2);

public:
    QPixmap *m_pPixmap;
    QGraphicsPixmapItem *m_pItem;
    QGraphicsScene *m_pScene;
    CGGraphicsView *m_pGraphicsView;

    bool bGraphicsScene = false;

    QLineF m_Line;
    QRectF m_Rect;

    enum ToolType
    {
        TwoPointLineTool,
        VerticalLineTool,
        HorizontalLineTool,
        RectTool,
        CircleTool,
        ArcTool
    };

    ToolType  m_CurrentToolType;
    ToolType  m_LastToolType;

private:
    void InitTwoPointLineTool();
    void InitRectTool();
    void InitCircleTool();
    void InitHorizontalLineTool();
    void InitVerticalLineTool();

private:
    CGGraphicsLineItem *m_pTwoPointLineTool;
    CGGraphicsRectItem *m_pRectTool;
    CGGraphicsCircleItem *m_pCircleTool;
    CGGraphicsLineItemHorizontal *m_pHorizontalLineTool;
    CGGraphicsLineItemVertical *m_pVerticalLineTool;

    bool IsTool = false;

};

#endif // CGPROFILEFORM2D_H
