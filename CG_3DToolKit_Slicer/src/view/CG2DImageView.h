#ifndef CG2DIMAGEVIEW_H
#define CG2DIMAGEVIEW_H

#include <CGBaseWidget.h>
#include <CGGraphicsView.h>

class CG2DImageView : public CGBaseWidget
{
    Q_OBJECT

public:
    explicit CG2DImageView(QWidget* parent = nullptr);
    ~CG2DImageView() = default;

signals:

public:
    void InitUi() override;
    void InitConnections() override;
    void LoadImages(const QString FileName);
    void ClearImages();

    QPixmap* GetPixmap() const;

public:
    QPixmap *m_pPixmap;
    QGraphicsPixmapItem *m_pItem;
    QGraphicsScene *m_pScene;
    CGGraphicsView *m_pGraphicsView;

    bool bGraphicsScene = false;

};

#endif // CG2DIMAGEVIEW_H
