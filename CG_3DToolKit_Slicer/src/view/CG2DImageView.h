#ifndef CG2DIMAGEVIEW_H
#define CG2DIMAGEVIEW_H

#include <CGBaseWidget.h>
#include <CGGraphicsView.h>
#include <CGImage2DGraphicsItemAdapter.h>

class CGGraphicsLineItem;
class CGGraphicsRectItem;
class CGGraphicsCircleItem;

class CG2DImageView : public CGBaseWidget
{
    Q_OBJECT

public:
    explicit CG2DImageView(QWidget* parent = nullptr);
    ~CG2DImageView() = default;

signals:

public slots:
    void OnUseTool();
    void OnDelTool();

public:
    void InitUi() override;
    void InitConnections() override;
    void LoadImages(const QString FileName);
    void LoadImages(const QPixmap Pixmap);
    void ClearImages();
    void InitTools();
    void RemoveTools();

    QPixmap* GetPixmap() const;

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

    void TwoPointLineProfileHandle();
    void RectProfileHandle();
    void CircleProfileHandle();

private:
    CGGraphicsLineItem *m_pTwoPointLineTool;
    CGGraphicsRectItem *m_pRectTool;
    CGGraphicsCircleItem *m_pCircleTool;

    bool IsTool = false;

};

#endif // CG2DIMAGEVIEW_H
